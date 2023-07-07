// Package viamcartographer implements simultaneous localization and mapping.
// This is an Experimental package.
package viamcartographer

import (
	"bufio"
	"context"
	"io"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	pb "go.viam.com/api/service/slam/v1"
	viamgrpc "go.viam.com/rdk/grpc"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/services/slam/grpchelper"
	"go.viam.com/rdk/spatialmath"
	goutils "go.viam.com/utils"
	"go.viam.com/utils/pexec"

	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	dim2d "github.com/viamrobotics/viam-cartographer/internal/dim-2d"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
	vcUtils "github.com/viamrobotics/viam-cartographer/utils"
)

// Model is the model name of cartographer.
var Model = resource.NewModel("viam", "slam", "cartographer")

const (
	// DefaultExecutableName is what this program expects to call to start the cartographer grpc server.
	DefaultExecutableName                = "carto_grpc_server"
	defaultDataRateMsec                  = 200
	defaultMapRateSec                    = 60
	defaultDialMaxTimeoutSec             = 30
	defaultSensorValidationMaxTimeoutSec = 30
	defaultSensorValidationIntervalSec   = 1
	parsePortMaxTimeoutSec               = 60
	localhost0                           = "localhost:0"
)

// SubAlgo defines the cartographer specific sub-algorithms that we support.
type SubAlgo string

// Dim2d runs cartographer with a 2D LIDAR only.
const Dim2d SubAlgo = "2d"

func init() {
	resource.RegisterService(slam.API, Model, resource.Registration[slam.Service, *vcConfig.Config]{
		Constructor: func(
			ctx context.Context,
			deps resource.Dependencies,
			c resource.Config,
			logger golog.Logger,
		) (slam.Service, error) {
			return New(
				ctx,
				deps,
				c,
				logger,
				false,
				DefaultExecutableName,
				defaultSensorValidationMaxTimeoutSec,
				defaultSensorValidationIntervalSec,
				defaultDialMaxTimeoutSec,
			)
		},
	})
}

// New returns a new slam service for the given robot.
func New(
	ctx context.Context,
	deps resource.Dependencies,
	c resource.Config,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
	executableName string,
	sensorValidationMaxTimeoutSec int,
	sensorValidationIntervalSec int,
	dialMaxTimeoutSec int,
) (slam.Service, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::slamService::New")
	defer span.End()

	svcConfig, err := resource.NativeConfig[*vcConfig.Config](c)
	if err != nil {
		return nil, err
	}

	subAlgo := SubAlgo(svcConfig.ConfigParams["mode"])
	if subAlgo != Dim2d {
		return nil, errors.Errorf("%v does not have a 'mode: %v'",
			c.Model.Name, svcConfig.ConfigParams["mode"])
	}

	// Set up the data directories
	if err := vcConfig.SetupDirectories(svcConfig.DataDirectory, logger); err != nil {
		return nil, err
	}

	port, dataRateMsec, mapRateSec, useLiveData, deleteProcessedData, _, err := vcConfig.GetOptionalParameters(
		svcConfig,
		localhost0,
		defaultDataRateMsec,
		defaultMapRateSec,
		logger,
	)
	if err != nil {
		return nil, err
	}

	// Get the lidar for the Dim2D cartographer sub algorithm
	lidar, err := dim2d.NewLidar(ctx, deps, svcConfig.Sensors, logger)
	if err != nil {
		return nil, err
	}

	// Need to pass in a long-lived context because ctx is short-lived
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	// Cartographer SLAM Service Object
	cartoSvc := &cartographerService{
		Named:                 c.ResourceName().AsNamed(),
		primarySensorName:     lidar.Name,
		executableName:        executableName,
		subAlgo:               subAlgo,
		slamProcess:           pexec.NewProcessManager(logger),
		configParams:          svcConfig.ConfigParams,
		dataDirectory:         svcConfig.DataDirectory,
		useLiveData:           useLiveData,
		deleteProcessedData:   deleteProcessedData,
		port:                  port,
		dataRateMs:            dataRateMsec,
		mapRateSec:            mapRateSec,
		cancelFunc:            cancelFunc,
		logger:                logger,
		bufferSLAMProcessLogs: bufferSLAMProcessLogs,
		localizationMode:      mapRateSec == 0,
		mapTimestamp:          time.Now().UTC(),
	}

	success := false
	defer func() {
		if !success {
			if err := cartoSvc.Close(ctx); err != nil {
				logger.Errorw("error closing out after error", "error", err)
			}
		}
	}()

	if cartoSvc.primarySensorName != "" {
		if err := dim2d.ValidateGetAndSaveData(cancelCtx, cartoSvc.dataDirectory, lidar,
			sensorValidationMaxTimeoutSec, sensorValidationIntervalSec, cartoSvc.logger); err != nil {
			return nil, errors.Wrap(err, "getting and saving data failed")
		}
		cartoSvc.StartDataProcess(cancelCtx, lidar, nil)
		logger.Debugf("Reading data from sensor: %v", cartoSvc.primarySensorName)
	}

	if err := cartoSvc.StartSLAMProcess(ctx); err != nil {
		return nil, errors.Wrap(err, "error with slam service slam process")
	}

	client, clientClose, err := vcConfig.SetupGRPCConnection(ctx, cartoSvc.port, dialMaxTimeoutSec, logger)
	if err != nil {
		return nil, errors.Wrap(err, "error with initial grpc client to slam algorithm")
	}
	cartoSvc.clientAlgo = client
	cartoSvc.clientAlgoClose = clientClose

	success = true
	return cartoSvc, nil
}

// cartographerService is the structure of the slam service.
type cartographerService struct {
	resource.Named
	resource.AlwaysRebuild
	primarySensorName string
	executableName    string
	subAlgo           SubAlgo
	slamProcess       pexec.ProcessManager
	clientAlgo        pb.SLAMServiceClient
	clientAlgoClose   func() error

	configParams        map[string]string
	dataDirectory       string
	deleteProcessedData bool
	useLiveData         bool

	port       string
	dataRateMs int
	mapRateSec int

	cancelFunc              func()
	logger                  golog.Logger
	activeBackgroundWorkers sync.WaitGroup

	bufferSLAMProcessLogs        bool
	slamProcessLogReader         io.ReadCloser
	slamProcessLogWriter         io.WriteCloser
	slamProcessBufferedLogReader bufio.Reader

	localizationMode bool
	mapTimestamp     time.Time
}

// GetPosition forwards the request for positional data to the slam library's gRPC service. Once a response is received,
// it is unpacked into a Pose and a component reference string.
func (cartoSvc *cartographerService) GetPosition(ctx context.Context) (spatialmath.Pose, string, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::GetPosition")
	defer span.End()

	req := &pb.GetPositionRequest{Name: cartoSvc.Name().ShortName()}

	resp, err := cartoSvc.clientAlgo.GetPosition(ctx, req)
	if err != nil {
		return nil, "", errors.Wrap(err, "error getting SLAM position")
	}
	pose := spatialmath.NewPoseFromProtobuf(resp.GetPose())
	componentReference := resp.GetComponentReference()
	returnedExt := resp.Extra.AsMap()

	return vcUtils.CheckQuaternionFromClientAlgo(pose, componentReference, returnedExt)
}

// GetPointCloudMap creates a request, recording the time, calls the slam algorithms GetPointCloudMap endpoint and returns a callback
// function which will return the next chunk of the current pointcloud map.
// If startup is in localization mode, the timestamp is NOT updated.
func (cartoSvc *cartographerService) GetPointCloudMap(ctx context.Context) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::GetPointCloudMap")
	defer span.End()

	if !cartoSvc.localizationMode {
		cartoSvc.mapTimestamp = time.Now().UTC()
	}
	return grpchelper.GetPointCloudMapCallback(ctx, cartoSvc.Name().ShortName(), cartoSvc.clientAlgo)
}

// GetInternalState creates a request, calls the slam algorithms GetInternalState endpoint and returns a callback
// function which will return the next chunk of the current internal state of the slam algo.
func (cartoSvc *cartographerService) GetInternalState(ctx context.Context) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::GetInternalState")
	defer span.End()

	return grpchelper.GetInternalStateCallback(ctx, cartoSvc.Name().ShortName(), cartoSvc.clientAlgo)
}

// GetLatestMapInfo returns the timestamp  associated with the latest call to GetPointCloudMap,
// unless you are localizing; in which case the timestamp returned is the timestamp of the session.
func (cartoSvc *cartographerService) GetLatestMapInfo(ctx context.Context) (time.Time, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::GetLatestMapInfo")
	defer span.End()

	return cartoSvc.mapTimestamp, nil
}

// StartDataProcess starts a go routine that saves data from the lidar to the user-defined data directory.
func (cartoSvc *cartographerService) StartDataProcess(
	ctx context.Context,
	lidar lidar.Lidar,
	c chan int,
) {
	cartoSvc.activeBackgroundWorkers.Add(1)
	if err := ctx.Err(); err != nil {
		if !errors.Is(err, context.Canceled) {
			cartoSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
		}
		cartoSvc.activeBackgroundWorkers.Done()
		return
	}

	goutils.PanicCapturingGo(func() {
		if !cartoSvc.useLiveData {
			// If we're not using live data, we read from the sensor as fast as
			// possible, since the sensor is just playing back pre-captured data.
			cartoSvc.readData(ctx, lidar, c)
		} else {
			cartoSvc.readDataOnInterval(ctx, lidar, c)
		}
	})
}

func (cartoSvc *cartographerService) readData(ctx context.Context, lidar lidar.Lidar, c chan int) {
	for {
		if err := ctx.Err(); err != nil {
			if !errors.Is(err, context.Canceled) {
				cartoSvc.logger.Errorw("unexpected error in SLAM data process", "error", err)
			}
			return
		}

		cartoSvc.getNextDataPoint(ctx, lidar, c)
	}
}

func (cartoSvc *cartographerService) readDataOnInterval(ctx context.Context, lidar lidar.Lidar, c chan int) {
	ticker := time.NewTicker(time.Millisecond * time.Duration(cartoSvc.dataRateMs))
	defer ticker.Stop()
	defer cartoSvc.activeBackgroundWorkers.Done()

	for {
		if err := ctx.Err(); err != nil {
			if !errors.Is(err, context.Canceled) {
				cartoSvc.logger.Errorw("unexpected error in SLAM data process", "error", err)
			}
			return
		}

		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			if err := ctx.Err(); err != nil {
				if !errors.Is(err, context.Canceled) {
					cartoSvc.logger.Errorw("unexpected error in SLAM data process", "error", err)
				}
				return
			}

			cartoSvc.activeBackgroundWorkers.Add(1)
			goutils.PanicCapturingGo(func() {
				defer cartoSvc.activeBackgroundWorkers.Done()
				cartoSvc.getNextDataPoint(ctx, lidar, c)
			})
		}
	}
}

func (cartoSvc *cartographerService) getNextDataPoint(ctx context.Context, lidar lidar.Lidar, c chan int) {
	if _, err := dim2d.GetAndSaveData(ctx, cartoSvc.dataDirectory, lidar, cartoSvc.logger); err != nil {
		cartoSvc.logger.Warn(err)
	}
	if c != nil {
		c <- 1
	}
}

func (cartoSvc *cartographerService) DoCommand(ctx context.Context, req map[string]interface{}) (map[string]interface{}, error) {
	return nil, viamgrpc.UnimplementedError
}

// Close out of all slam related processes.
func (cartoSvc *cartographerService) Close(ctx context.Context) error {
	defer func() {
		if cartoSvc.clientAlgoClose != nil {
			goutils.UncheckedErrorFunc(cartoSvc.clientAlgoClose)
		}
	}()
	cartoSvc.cancelFunc()
	if cartoSvc.bufferSLAMProcessLogs {
		if cartoSvc.slamProcessLogReader != nil {
			if err := cartoSvc.slamProcessLogReader.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log reader")
			}
		}
		if cartoSvc.slamProcessLogWriter != nil {
			if err := cartoSvc.slamProcessLogWriter.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log writer")
			}
		}
	}
	if err := cartoSvc.StopSLAMProcess(); err != nil {
		return errors.Wrap(err, "error occurred during closeout of process")
	}
	cartoSvc.activeBackgroundWorkers.Wait()
	return nil
}

// GetSLAMProcessConfig returns the process config for the SLAM process.
func (cartoSvc *cartographerService) GetSLAMProcessConfig() pexec.ProcessConfig {
	var args []string

	args = append(args, "-sensors="+cartoSvc.primarySensorName)
	args = append(args, "-config_param="+vcUtils.DictToString(cartoSvc.configParams))
	args = append(args, "-data_rate_ms="+strconv.Itoa(cartoSvc.dataRateMs))
	args = append(args, "-map_rate_sec="+strconv.Itoa(cartoSvc.mapRateSec))
	args = append(args, "-data_dir="+cartoSvc.dataDirectory)
	args = append(args, "-delete_processed_data="+strconv.FormatBool(cartoSvc.deleteProcessedData))
	args = append(args, "-use_live_data="+strconv.FormatBool(cartoSvc.useLiveData))
	args = append(args, "-port="+cartoSvc.port)
	args = append(args, "--aix-auto-update")

	target := cartoSvc.executableName

	appDir := os.Getenv("APPDIR")

	if appDir != "" {
		// The carto grpc server is expected to be in
		// /usr/bin/ if we are running in an AppImage
		target = appDir + "/usr/bin/" + strings.TrimPrefix(target, "/")
	}

	return pexec.ProcessConfig{
		ID:   "slam_cartographer",
		Name: target,
		Args: args,
		Log:  true,
		// In appimage this is set to the appimage
		// squashfs mount location (/tmp/.mountXXXXX)
		// Otherwise, it is an empty string
		CWD:     os.Getenv("APPRUN_RUNTIME"),
		OneShot: false,
	}
}

func (cartoSvc *cartographerService) GetSLAMProcessBufferedLogReader() bufio.Reader {
	return cartoSvc.slamProcessBufferedLogReader
}

// startSLAMProcess starts up the SLAM library process by calling the executable binary and giving it the necessary arguments.
func (cartoSvc *cartographerService) StartSLAMProcess(ctx context.Context) error {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::slamService::StartSLAMProcess")
	defer span.End()

	processConfig := cartoSvc.GetSLAMProcessConfig()

	var logReader io.ReadCloser
	var logWriter io.WriteCloser
	var bufferedLogReader bufio.Reader
	if cartoSvc.port == localhost0 || cartoSvc.bufferSLAMProcessLogs {
		logReader, logWriter = io.Pipe()
		bufferedLogReader = *bufio.NewReader(logReader)
		processConfig.LogWriter = logWriter
	}

	_, err := cartoSvc.slamProcess.AddProcessFromConfig(ctx, processConfig)
	if err != nil {
		return errors.Wrap(err, "problem adding slam process")
	}

	cartoSvc.logger.Debug("starting slam process")

	if err = cartoSvc.slamProcess.Start(ctx); err != nil {
		return errors.Wrap(err, "problem starting slam process")
	}

	if cartoSvc.port == localhost0 {
		timeoutCtx, timeoutCancel := context.WithTimeout(ctx, parsePortMaxTimeoutSec*time.Second)
		defer timeoutCancel()

		if !cartoSvc.bufferSLAMProcessLogs {
			defer func(logger golog.Logger) {
				if err := logReader.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(cartoSvc.logger)
			defer func(logger golog.Logger) {
				if err := logWriter.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(cartoSvc.logger)
		}

		for {
			if err := timeoutCtx.Err(); err != nil {
				return errors.Wrapf(err, "error getting port from slam process")
			}

			line, err := bufferedLogReader.ReadString('\n')
			if err != nil {
				return errors.Wrapf(err, "error getting port from slam process")
			}
			portLogLinePrefix := "Server listening on "
			if strings.Contains(line, portLogLinePrefix) {
				linePieces := strings.Split(line, portLogLinePrefix)
				if len(linePieces) != 2 {
					return errors.Errorf("failed to parse port from slam process log line: %v", line)
				}
				cartoSvc.port = "localhost:" + strings.TrimRight(linePieces[1], "\n")
				break
			}
		}
	}

	if cartoSvc.bufferSLAMProcessLogs {
		cartoSvc.slamProcessLogReader = logReader
		cartoSvc.slamProcessLogWriter = logWriter
		cartoSvc.slamProcessBufferedLogReader = bufferedLogReader
	}

	return nil
}

// stopSLAMProcess uses the process manager to stop the created slam process from running.
func (cartoSvc *cartographerService) StopSLAMProcess() error {
	if err := cartoSvc.slamProcess.Stop(); err != nil {
		return errors.Wrap(err, "problem stopping slam process")
	}
	return nil
}
