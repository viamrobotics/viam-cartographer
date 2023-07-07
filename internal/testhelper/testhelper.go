// Package testhelper implements a slam service definition with additional exported functions for
// the purpose of testing. It does also contain helper variables and functions that are
// used across all tests in the viam-cartographer repo.
package testhelper

import (
	"bufio"
	"context"
	"os"
	"strconv"
	"sync/atomic"
	"testing"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"github.com/viamrobotics/gostream"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/utils/contextutils"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"
	"go.viam.com/utils/pexec"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
)

const (
	// NumPointClouds is the number of pointclouds saved in artifact
	// for the cartographer integration tests.
	NumPointClouds = 15
	// SensorValidationMaxTimeoutSecForTest is used in the ValidateGetAndSaveData
	// function to ensure that the sensor in the GetAndSaveData function
	// returns data within an acceptable time.
	SensorValidationMaxTimeoutSecForTest = 1
	// SensorValidationIntervalSecForTest is used in the ValidateGetAndSaveData
	// function for the while loop that attempts to grab data from the
	// sensor that is used in the GetAndSaveData function.
	SensorValidationIntervalSecForTest = 1
	testDialMaxTimeoutSec              = 1
	// TestTime can be used to test specific timestamps provided by a replay sensor.
	TestTime = "2006-01-02T15:04:05.9999Z"
)

// IntegrationLidarReleasePointCloudChan is the lidar pointcloud release
// channel for the integration tests.
var IntegrationLidarReleasePointCloudChan = make(chan int, 1)

// Service in the internal package includes additional exported functions relating to the data and
// slam processes in the slam service. These functions are not exported to the user. This resolves
// a circular import caused by the inject package.
type Service interface {
	StartDataProcess(cancelCtx context.Context, lidar lidar.Lidar, c chan int)
	StartSLAMProcess(ctx context.Context) error
	StopSLAMProcess() error
	Close(ctx context.Context) error
	GetSLAMProcessConfig() pexec.ProcessConfig
	GetSLAMProcessBufferedLogReader() bufio.Reader
}

// SetupDeps returns the dependencies based on the sensors passed as arguments.
func SetupDeps(sensors []string) resource.Dependencies {
	deps := make(resource.Dependencies)

	for _, sensor := range sensors {
		switch sensor {
		case "good_lidar":
			deps[camera.Named(sensor)] = getGoodLidar()
		case "replay_sensor":
			deps[camera.Named(sensor)] = getReplaySensor()
		case "invalid_sensor":
			deps[camera.Named(sensor)] = getInvalidSensor()
		case "gibberish":
			return deps
		case "cartographer_int_lidar":
			deps[camera.Named(sensor)] = getIntegrationLidar()
		default:
			continue
		}
	}
	return deps
}

func getGoodLidar() *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return pointcloud.New(), nil
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	return cam
}

func getReplaySensor() *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{TestTime}
		}
		return pointcloud.New(), nil
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	return cam
}

func getInvalidSensor() *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return nil, errors.New("invalid sensor")
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("invalid sensor")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	return cam
}

func getIntegrationLidar() *inject.Camera {
	cam := &inject.Camera{}
	var index uint64
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		select {
		case <-IntegrationLidarReleasePointCloudChan:
			i := atomic.AddUint64(&index, 1) - 1
			if i >= NumPointClouds {
				return nil, errors.New("No more cartographer point clouds")
			}
			file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/" + strconv.FormatUint(i, 10) + ".pcd"))
			if err != nil {
				return nil, err
			}
			pointCloud, err := pointcloud.ReadPCD(file)
			if err != nil {
				return nil, err
			}
			return pointCloud, nil
		default:
			return nil, errors.Errorf("Lidar not ready to return point cloud %v", atomic.LoadUint64(&index))
		}
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	return cam
}

// ClearDirectory deletes the contents in the path directory
// without deleting path itself.
func ClearDirectory(t *testing.T, path string) {
	t.Helper()

	err := ResetFolder(path)
	test.That(t, err, test.ShouldBeNil)
}

// CreateSLAMService creates a slam service for testing.
func CreateSLAMService(
	t *testing.T,
	cfg *vcConfig.Config,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
	executableName string,
) (slam.Service, error) {
	t.Helper()

	ctx := context.Background()
	cfgService := resource.Config{Name: "test", API: slam.API, Model: viamcartographer.Model}
	cfgService.ConvertedAttributes = cfg

	deps := SetupDeps(cfg.Sensors)

	sensorDeps, err := cfg.Validate("path")
	if err != nil {
		return nil, err
	}
	test.That(t, sensorDeps, test.ShouldResemble, cfg.Sensors)

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		bufferSLAMProcessLogs,
		executableName,
		SensorValidationMaxTimeoutSecForTest,
		SensorValidationIntervalSecForTest,
		testDialMaxTimeoutSec,
	)
	if err != nil {
		test.That(t, svc, test.ShouldBeNil)
		return nil, err
	}

	test.That(t, svc, test.ShouldNotBeNil)

	return svc, nil
}

// CheckDeleteProcessedData compares the number of files found in a specified data
// directory with the previous number found and uses the useLiveData and
// deleteProcessedData values to evaluate this comparison. It returns the number of files
// currently in the data directory for the specified config. Future invocations should pass in this
// value. This function should be passed 0 as a default prev argument in order to get the
// number of files currently in the directory.
func CheckDeleteProcessedData(
	t *testing.T,
	subAlgo viamcartographer.SubAlgo,
	dir string,
	prev int,
	deleteProcessedData,
	useLiveData bool,
) int {
	switch subAlgo {
	case viamcartographer.Dim2d:
		numFiles, err := CheckDataDirForExpectedFiles(t, dir+"/data", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)
		return numFiles
	default:
		return 0
	}
}

const (
	dataBufferSize = 4
)

// CreateTempFolderArchitecture creates a new random temporary
// directory with the config, data, and map subdirectories needed
// to run the SLAM libraries.
func CreateTempFolderArchitecture(logger golog.Logger) (string, error) {
	tmpDir, err := os.MkdirTemp("", "*")
	if err != nil {
		return "nil", err
	}
	if err := vcConfig.SetupDirectories(tmpDir, logger); err != nil {
		return "", err
	}
	return tmpDir, nil
}

// ResetFolder removes all content in path and creates a new directory
// in its place.
func ResetFolder(path string) error {
	dirInfo, err := os.Stat(path)
	if err != nil {
		return err
	}
	if !dirInfo.IsDir() {
		return errors.Errorf("the path passed ResetFolder does not point to a folder: %v", path)
	}
	if err = os.RemoveAll(path); err != nil {
		return err
	}
	return os.Mkdir(path, dirInfo.Mode())
}

// CheckDataDirForExpectedFiles ensures that the provided data directory contains the correct amount of files
// based on the config parameters deleteProcessedData and useLiveData.
func CheckDataDirForExpectedFiles(t *testing.T, dir string, prev int, deleteProcessedData, useLiveData bool) (int, error) {
	files, err := os.ReadDir(dir)
	test.That(t, err, test.ShouldBeNil)

	if prev == 0 {
		return len(files), nil
	}
	if deleteProcessedData && useLiveData {
		test.That(t, prev, test.ShouldBeLessThanOrEqualTo, dataBufferSize+1)
	}
	if !deleteProcessedData && useLiveData {
		test.That(t, prev, test.ShouldBeLessThan, len(files))
	}
	if deleteProcessedData && !useLiveData {
		return 0, errors.New("the delete_processed_data value cannot be true when running SLAM in offline mode")
	}
	if !deleteProcessedData && !useLiveData {
		test.That(t, prev, test.ShouldEqual, len(files))
	}
	return len(files), nil
}
