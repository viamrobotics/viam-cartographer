// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user. It also runs integration tests
// that test the interaction with the core C++ viam-cartographer code and the Golang implementation of the
// cartographer slam service.
package viamcartographer_test

import (
	"context"
	"fmt"
	"net"
	"os"
	"path/filepath"
	"strconv"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	viamgrpc "go.viam.com/rdk/grpc"
	"go.viam.com/test"
	"google.golang.org/grpc"

	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
)

const (
	testExecutableName = "true" // the program "true", not the boolean value
	testDataRateMsec   = 200
)

var (
	testMapRateSec = 200
	_true          = true
	_false         = false
	_zeroInt       = 0
	_zeroTime      = time.Time{}
)

func TestNew(t *testing.T) {
	logger := golog.NewTestLogger(t)
	dataDir, err := testhelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Successful creation of cartographer slam service with no sensor", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		test.That(t, err, test.ShouldBeNil)
		attrCfg := &vcConfig.Config{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		grpcServer.Stop()
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with more than one sensor", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		test.That(t, err, test.ShouldBeNil)
		attrCfg := &vcConfig.Config{
			Sensors:       []string{"lidar", "one-too-many"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		_, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: 'sensors' must contain only one "+
				"lidar camera, but is 'sensors: [lidar, one-too-many]'"))

		grpcServer.Stop()
	})

	t.Run("Failed creation of cartographer slam service with non-existing sensor", func(t *testing.T) {
		attrCfg := &vcConfig.Config{
			Sensors:       []string{"gibberish"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			DataRateMsec:  testDataRateMsec,
			UseLiveData:   &_true,
		}

		_, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: error getting lidar camera "+
				"gibberish for slam service: \"rdk:component:camera/gibberish\" missing from dependencies"))
	})

	t.Run("Successful creation of cartographer slam service with good lidar", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &vcConfig.Config{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			DataRateMsec:  testDataRateMsec,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		grpcServer.Stop()
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with invalid sensor "+
		"that errors during call to NextPointCloud", func(t *testing.T) {
		attrCfg := &vcConfig.Config{
			Sensors:       []string{"invalid_sensor"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			DataRateMsec:  testDataRateMsec,
			UseLiveData:   &_true,
		}

		_, err = testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("getting and saving data failed: error getting data from sensor: invalid sensor"))
	})

	testhelper.ClearDirectory(t, dataDir)

	t.Run("Successful creation of cartographer slam service in localization mode", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)

		attrCfg := &vcConfig.Config{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
			MapRateSec:    &_zeroInt,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		timestamp1, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)
		_, err = svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)
		timestamp2, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp1, test.ShouldResemble, timestamp2)

		grpcServer.Stop()
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service in non localization mode", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &vcConfig.Config{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
			MapRateSec:    &testMapRateSec,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		timestamp1, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)
		_, err = svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)
		timestamp2, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp2.After(timestamp1), test.ShouldBeTrue)

		grpcServer.Stop()
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})
}

func TestDataProcess(t *testing.T) {
	logger, obs := golog.NewObservedTestLogger(t)
	dataDir, err := testhelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &vcConfig.Config{
		Sensors:       []string{"good_lidar"},
		ConfigParams:  map[string]string{"mode": "2d"},
		DataDirectory: dataDir,
		DataRateMsec:  testDataRateMsec,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
	test.That(t, err, test.ShouldBeNil)

	grpcServer.Stop()
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

	slamSvc := svc.(testhelper.Service)

	t.Run("Successful startup of data process with good lidar", func(t *testing.T) {
		defer testhelper.ClearDirectory(t, filepath.Join(dataDir, "data"))

		sensors := []string{"good_lidar"}
		lidar, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		c := make(chan int, 100)
		slamSvc.StartDataProcess(cancelCtx, lidar, c)

		<-c
		cancelFunc()
		files, err := os.ReadDir(filepath.Join(dataDir, "data"))
		test.That(t, len(files), test.ShouldBeGreaterThanOrEqualTo, 1)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Failed startup of data process with invalid sensor "+
		"that errors during call to NextPointCloud", func(t *testing.T) {
		sensors := []string{"invalid_sensor"}
		lidar, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		c := make(chan int, 100)
		slamSvc.StartDataProcess(cancelCtx, lidar, c)

		<-c
		allObs := obs.All()
		latestLoggedEntry := allObs[len(allObs)-1]
		cancelFunc()
		test.That(t, fmt.Sprint(latestLoggedEntry), test.ShouldContainSubstring, "invalid sensor")
	})

	t.Run("When replay sensor is configured, we read timestamps from the context", func(t *testing.T) {
		defer testhelper.ClearDirectory(t, filepath.Join(dataDir, "data"))

		sensors := []string{"replay_sensor"}
		lidar, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		c := make(chan int, 100)
		slamSvc.StartDataProcess(cancelCtx, lidar, c)

		<-c
		cancelFunc()
		files, err := os.ReadDir(filepath.Join(dataDir, "data"))
		test.That(t, len(files), test.ShouldBeGreaterThanOrEqualTo, 1)
		test.That(t, files[0].Name(), test.ShouldContainSubstring, testhelper.TestTime)
		test.That(t, err, test.ShouldBeNil)
	})

	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

	testhelper.ClearDirectory(t, dataDir)
}

func TestEndpointFailures(t *testing.T) {
	logger := golog.NewTestLogger(t)
	dataDir, err := testhelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &vcConfig.Config{
		Sensors:       []string{"good_lidar"},
		ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
		DataDirectory: dataDir,
		MapRateSec:    &testMapRateSec,
		DataRateMsec:  testDataRateMsec,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
	test.That(t, err, test.ShouldBeNil)

	pNew, frame, err := svc.GetPosition(context.Background())
	test.That(t, pNew, test.ShouldBeNil)
	test.That(t, frame, test.ShouldBeEmpty)
	test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "error getting SLAM position")

	callbackPointCloud, err := svc.GetPointCloudMap(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, callbackPointCloud, test.ShouldNotBeNil)
	chunkPCD, err := callbackPointCloud()
	test.That(t, err.Error(), test.ShouldContainSubstring, "error receiving pointcloud chunk")
	test.That(t, chunkPCD, test.ShouldBeNil)

	callbackInternalState, err := svc.GetInternalState(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, callbackInternalState, test.ShouldNotBeNil)
	chunkInternalState, err := callbackInternalState()
	test.That(t, err.Error(), test.ShouldContainSubstring, "error receiving internal state chunk")
	test.That(t, chunkInternalState, test.ShouldBeNil)

	grpcServer.Stop()
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

	testhelper.ClearDirectory(t, dataDir)
}

func TestSLAMProcess(t *testing.T) {
	logger := golog.NewTestLogger(t)
	dataDir, err := testhelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Successful start of live SLAM process with default parameters", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &vcConfig.Config{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		slamSvc := svc.(testhelper.Service)
		processCfg := slamSvc.GetSLAMProcessConfig()
		cmd := append([]string{processCfg.Name}, processCfg.Args...)

		cmdResult := [][]string{
			{testExecutableName},
			{"-sensors=good_lidar"},
			{"-config_param={test_param=viam,mode=2d}", "-config_param={mode=2d,test_param=viam}"},
			{"-data_rate_ms=200"},
			{"-map_rate_sec=60"},
			{"-data_dir=" + dataDir},
			{"-delete_processed_data=true"},
			{"-use_live_data=true"},
			{"-port=localhost:" + strconv.Itoa(port)},
			{"--aix-auto-update"},
		}

		for i, s := range cmd {
			t.Run(fmt.Sprintf("Test command argument %v at index %v", s, i), func(t *testing.T) {
				test.That(t, s, test.ShouldBeIn, cmdResult[i])
			})
		}

		grpcServer.Stop()
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful start of offline SLAM process with default parameters", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &vcConfig.Config{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		slamSvc := svc.(testhelper.Service)
		processCfg := slamSvc.GetSLAMProcessConfig()
		cmd := append([]string{processCfg.Name}, processCfg.Args...)

		cmdResult := [][]string{
			{testExecutableName},
			{"-sensors="},
			{"-config_param={mode=2d,test_param=viam}", "-config_param={test_param=viam,mode=2d}"},
			{"-data_rate_ms=200"},
			{"-map_rate_sec=60"},
			{"-data_dir=" + dataDir},
			{"-delete_processed_data=false"},
			{"-use_live_data=false"},
			{"-port=localhost:" + strconv.Itoa(port)},
			{"--aix-auto-update"},
		}

		for i, s := range cmd {
			t.Run(fmt.Sprintf("Test command argument %v at index %v", s, i), func(t *testing.T) {
				test.That(t, s, test.ShouldBeIn, cmdResult[i])
			})
		}

		grpcServer.Stop()
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Failed start of SLAM process that errors out due to invalid binary location", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &vcConfig.Config{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
			DataDirectory: dataDir,
			MapRateSec:    &testMapRateSec,
			DataRateMsec:  testDataRateMsec,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}
		_, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, "hokus_pokus_does_not_exist_filename")
		test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "executable file not found in $PATH")
		grpcServer.Stop()
	})

	testhelper.ClearDirectory(t, dataDir)
}

func TestDoCommand(t *testing.T) {
	logger := golog.NewTestLogger(t)
	dataDir, err := testhelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)
	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &vcConfig.Config{
		Sensors:       []string{"good_lidar"},
		ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
		DataDirectory: dataDir,
		MapRateSec:    &testMapRateSec,
		DataRateMsec:  testDataRateMsec,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}
	svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, false, testExecutableName)
	test.That(t, err, test.ShouldBeNil)
	t.Run("returns UnimplementedError when given other parmeters", func(t *testing.T) {
		cmd := map[string]interface{}{"fake_flag": true}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldEqual, viamgrpc.UnimplementedError)
		test.That(t, resp, test.ShouldBeNil)
	})
	t.Run("returns UnimplementedError when given no parmeters", func(t *testing.T) {
		cmd := map[string]interface{}{}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldEqual, viamgrpc.UnimplementedError)
		test.That(t, resp, test.ShouldBeNil)
	})
	grpcServer.Stop()
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	testhelper.ClearDirectory(t, dataDir)
}

// SetupTestGRPCServer sets up and starts a grpc server.
// It returns the grpc server and the port at which it is served.
func setupTestGRPCServer(tb testing.TB) (*grpc.Server, int) {
	listener, err := net.Listen("tcp", ":0")
	test.That(tb, err, test.ShouldBeNil)
	grpcServer := grpc.NewServer()
	go grpcServer.Serve(listener)

	return grpcServer, listener.Addr().(*net.TCPAddr).Port
}
