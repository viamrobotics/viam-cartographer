// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user. It also runs integration tests
// that test the interaction with the core C++ viam-cartographer code and the Golang implementation of the
// cartographer slam service.
package viamcartographer_test

import (
	"bytes"
	"context"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	"github.com/viamrobotics/viam-cartographer/dataprocess"
	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
)

const (
	cartoSleepMsec = 100
)

// Checks the cartographer map and confirms there at least 100 map points.
func testCartographerMap(t *testing.T, svc slam.Service, localizationMode bool) {
	timestamp1, err := svc.GetLatestMapInfo(context.Background())
	test.That(t, err, test.ShouldBeNil)
	pcd, err := slam.GetPointCloudMapFull(context.Background(), svc)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pcd, test.ShouldNotBeNil)
	timestamp2, err := svc.GetLatestMapInfo(context.Background())
	test.That(t, err, test.ShouldBeNil)

	if localizationMode == true {
		test.That(t, timestamp1, test.ShouldResemble, timestamp2)
	} else {
		test.That(t, timestamp2.After(timestamp1), test.ShouldBeTrue)
	}

	pointcloud, _ := pointcloud.ReadPCD(bytes.NewReader(pcd))
	t.Logf("Pointcloud points: %v", pointcloud.Size())
	test.That(t, pointcloud.Size(), test.ShouldBeGreaterThanOrEqualTo, 100)
}

// Checks the cartographer position within a defined tolerance.
func testCartographerPosition(t *testing.T, svc slam.Service, expectedComponentRef string) {
	expectedPos := r3.Vector{X: -4, Y: -4, Z: 0}
	tolerancePos := 40.0
	expectedOri := &spatialmath.R4AA{Theta: 0, RX: 0, RY: 0, RZ: -1}
	toleranceOri := 0.5

	position, componentRef, err := svc.GetPosition(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, componentRef, test.ShouldEqual, expectedComponentRef)

	actualPos := position.Point()
	t.Logf("Position point: (%v, %v, %v)", actualPos.X, actualPos.Y, actualPos.Z)
	test.That(t, actualPos.X, test.ShouldBeBetween, expectedPos.X-tolerancePos, expectedPos.X+tolerancePos)
	test.That(t, actualPos.Y, test.ShouldBeBetween, expectedPos.Y-tolerancePos, expectedPos.Y+tolerancePos)
	test.That(t, actualPos.Z, test.ShouldBeBetween, expectedPos.Z-tolerancePos, expectedPos.Z+tolerancePos)

	actualOri := position.Orientation().AxisAngles()
	t.Logf("Position orientation: RX: %v, RY: %v, RZ: %v, Theta: %v", actualOri.RX, actualOri.RY, actualOri.RZ, actualOri.Theta)
	test.That(t, actualOri.RX, test.ShouldBeBetween, expectedOri.RX-toleranceOri, expectedOri.RX+toleranceOri)
	test.That(t, actualOri.RY, test.ShouldBeBetween, expectedOri.RY-toleranceOri, expectedOri.RY+toleranceOri)
	test.That(t, actualOri.RZ, test.ShouldBeBetween, expectedOri.RZ-toleranceOri, expectedOri.RZ+toleranceOri)
	test.That(t, actualOri.Theta, test.ShouldBeBetween, expectedOri.Theta-toleranceOri, expectedOri.Theta+toleranceOri)
}

// Checks the cartographer internal state.
func testCartographerInternalState(t *testing.T, svc slam.Service, dataDir string) {
	internalState, err := slam.GetInternalStateFull(context.Background(), svc)
	test.That(t, err, test.ShouldBeNil)

	// Save the data from the call to GetInternalState for use in next test.
	timeStamp := time.Now()
	filename := filepath.Join(dataDir, "map", "map_data_"+timeStamp.UTC().Format(dataprocess.SlamTimeFormat)+".pbstream")
	err = os.WriteFile(filename, internalState, 0o644)
	test.That(t, err, test.ShouldBeNil)
}

func integrationtestHelperCartographer(t *testing.T, subAlgo viamcartographer.SubAlgo) {
	logger := golog.NewTestLogger(t)
	_, err := exec.LookPath("carto_grpc_server")
	if err != nil {
		t.Log("Skipping test because carto_grpc_server binary was not found")
		t.Skip()
	}

	dataDir, err := testhelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	prevNumFiles := 0

	t.Log("\n=== Testing online mode ===\n")

	mapRateSec := 9999
	deleteProcessedData := false
	useLiveData := true

	attrCfg := &vcConfig.Config{
		Sensors: []string{"cartographer_int_lidar"},
		ConfigParams: map[string]string{
			"mode":  reflect.ValueOf(subAlgo).String(),
			"v":     "1",
			"debug": "true",
		},
		MapRateSec:          &mapRateSec,
		DataDirectory:       dataDir,
		DeleteProcessedData: &deleteProcessedData,
		UseLiveData:         &useLiveData,
	}

	// Release point cloud for service validation
	testhelper.IntegrationLidarReleasePointCloudChan <- 1
	// Create slam service using a real cartographer binary
	svc, err := testhelper.CreateSLAMService(t, attrCfg, logger, true, viamcartographer.DefaultExecutableName)
	test.That(t, err, test.ShouldBeNil)

	// Release point cloud, since cartographer looks for the second most recent point cloud
	testhelper.IntegrationLidarReleasePointCloudChan <- 1

	// Make sure we initialize in mapping mode
	logReader := svc.(testhelper.Service).GetSLAMProcessBufferedLogReader()
	for {
		line, err := logReader.ReadString('\n')
		test.That(t, err, test.ShouldBeNil)
		if strings.Contains(line, "Running in mapping mode") {
			break
		}

		test.That(t, strings.Contains(line, "Running in updating mode"), test.ShouldBeFalse)
		test.That(t, strings.Contains(line, "Running in localization only mode"), test.ShouldBeFalse)
	}

	// Wait for cartographer to finish processing data
	for i := 0; i < testhelper.NumPointClouds-2; i++ {
		t.Logf("Find log line for point cloud %v", i)
		testhelper.IntegrationLidarReleasePointCloudChan <- 1
		for {
			line, err := logReader.ReadString('\n')
			test.That(t, err, test.ShouldBeNil)
			if strings.Contains(line, "Passed sensor data to SLAM") {
				prevNumFiles = testhelper.CheckDeleteProcessedData(
					t,
					subAlgo,
					dataDir,
					prevNumFiles,
					deleteProcessedData,
					useLiveData)
				break
			}
		}
	}

	testCartographerPosition(t, svc, attrCfg.Sensors[0])
	testCartographerMap(t, svc, false)

	// Close out slam service
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

	// Sleep to ensure cartographer stops
	time.Sleep(time.Millisecond * cartoSleepMsec)

	// Delete the last .pcd file in the data directory, so that offline mode runs on the
	// same data as online mode. (Online mode will not read the last .pcd file, since it
	// always processes the second-most-recent .pcd file, in case the most-recent .pcd
	// file is currently being written.)
	files, err := os.ReadDir(dataDir + "/data/")
	test.That(t, err, test.ShouldBeNil)
	lastFileName := files[len(files)-1].Name()
	test.That(t, os.Remove(dataDir+"/data/"+lastFileName), test.ShouldBeNil)
	prevNumFiles--

	// Check that no maps were generated during previous test
	testCartographerDir(t, dataDir, 0)

	// Test offline mode using the data generated in the online test
	t.Log("\n=== Testing offline mode ===\n")

	useLiveData = false
	mapRateSec = 1

	attrCfg = &vcConfig.Config{
		Sensors: []string{},
		ConfigParams: map[string]string{
			"mode": reflect.ValueOf(subAlgo).String(),
			"v":    "1",
		},
		MapRateSec:          &mapRateSec,
		DataDirectory:       dataDir,
		DeleteProcessedData: &deleteProcessedData,
		UseLiveData:         &useLiveData,
	}

	// Create slam service using a real cartographer binary
	svc, err = testhelper.CreateSLAMService(t, attrCfg, golog.NewTestLogger(t), true, viamcartographer.DefaultExecutableName)
	test.That(t, err, test.ShouldBeNil)

	// Make sure we initialize in mapping mode
	logReader = svc.(testhelper.Service).GetSLAMProcessBufferedLogReader()
	for {
		line, err := logReader.ReadString('\n')
		test.That(t, err, test.ShouldBeNil)
		if strings.Contains(line, "Running in mapping mode") {
			break
		}
		test.That(t, strings.Contains(line, "Running in updating mode"), test.ShouldBeFalse)
		test.That(t, strings.Contains(line, "Running in localization only mode"), test.ShouldBeFalse)
	}

	// Wait for cartographer to finish processing data
	for {
		line, err := logReader.ReadString('\n')
		test.That(t, err, test.ShouldBeNil)
		if strings.Contains(line, "Passed sensor data to SLAM") {
			prevNumFiles = testhelper.CheckDeleteProcessedData(
				t,
				subAlgo,
				dataDir,
				prevNumFiles,
				deleteProcessedData,
				useLiveData)
		}
		if strings.Contains(line, "No new data found") {
			break
		}
	}

	testCartographerPosition(t, svc, "") // leaving this empty because cartographer does not interpret the component reference in offline mode
	testCartographerMap(t, svc, false)

	// Sleep to ensure cartographer saves at least one map
	time.Sleep(time.Second * time.Duration(*attrCfg.MapRateSec))

	// Close out slam service
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

	// Sleep to ensure cartographer stops
	time.Sleep(time.Millisecond * cartoSleepMsec)

	// Remove existing pointclouds, but leave maps and config (so we keep the lua files).
	test.That(t, testhelper.ResetFolder(dataDir+"/data"), test.ShouldBeNil)
	prevNumFiles = 0

	// Count the initial number of maps in the map directory (should equal 1)
	testCartographerDir(t, dataDir, 1)

	// Test online mode using the map generated in the offline test
	t.Log("\n=== Testing online localization mode ===\n")

	mapRateSec = 0
	deleteProcessedData = true
	useLiveData = true

	attrCfg = &vcConfig.Config{
		Sensors: []string{"cartographer_int_lidar"},
		ConfigParams: map[string]string{
			"mode": reflect.ValueOf(subAlgo).String(),
			"v":    "1",
		},
		MapRateSec:          &mapRateSec,
		DataDirectory:       dataDir,
		DeleteProcessedData: &deleteProcessedData,
		UseLiveData:         &useLiveData,
	}

	// Release point cloud for service validation
	testhelper.IntegrationLidarReleasePointCloudChan <- 1
	// Create slam service using a real cartographer binary
	svc, err = testhelper.CreateSLAMService(t, attrCfg, golog.NewTestLogger(t), true, viamcartographer.DefaultExecutableName)
	test.That(t, err, test.ShouldBeNil)

	// Make sure we initialize in localization mode
	logReader = svc.(testhelper.Service).GetSLAMProcessBufferedLogReader()
	for {
		line, err := logReader.ReadString('\n')
		test.That(t, err, test.ShouldBeNil)
		if strings.Contains(line, "Running in localization only mode") {
			break
		}
		test.That(t, strings.Contains(line, "Running in updating mode"), test.ShouldBeFalse)
		test.That(t, strings.Contains(line, "Running in mapping mode"), test.ShouldBeFalse)
	}

	// Release point cloud, since cartographer looks for the second most recent point cloud
	testhelper.IntegrationLidarReleasePointCloudChan <- 1
	for i := 0; i < testhelper.NumPointClouds-2; i++ {
		t.Logf("Find log line for point cloud %v", i)
		testhelper.IntegrationLidarReleasePointCloudChan <- 1
		for {
			line, err := logReader.ReadString('\n')
			test.That(t, err, test.ShouldBeNil)
			if strings.Contains(line, "Passed sensor data to SLAM") {
				prevNumFiles = testhelper.CheckDeleteProcessedData(
					t,
					subAlgo,
					dataDir,
					prevNumFiles,
					deleteProcessedData,
					useLiveData)
				break
			}
		}
	}

	testCartographerPosition(t, svc, attrCfg.Sensors[0])
	testCartographerMap(t, svc, true)

	// Remove maps so that testing is done on the map generated by the internal map
	test.That(t, testhelper.ResetFolder(dataDir+"/map"), test.ShouldBeNil)

	testCartographerInternalState(t, svc, dataDir)

	// Close out slam service
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

	// Test that only the map present is the one generated by the GetInternalState call
	testCartographerDir(t, dataDir, 1)

	// Sleep to ensure cartographer stops
	time.Sleep(time.Millisecond * cartoSleepMsec)

	// Remove existing pointclouds, but leave maps and config (so we keep the lua files).
	test.That(t, testhelper.ResetFolder(dataDir+"/data"), test.ShouldBeNil)
	prevNumFiles = 0

	// Test online mode using the map generated in the offline test
	t.Log("\n=== Testing online mode with saved map ===\n")

	mapRateSec = 1

	attrCfg = &vcConfig.Config{
		Sensors: []string{"cartographer_int_lidar"},
		ConfigParams: map[string]string{
			"mode": reflect.ValueOf(subAlgo).String(),
			"v":    "1",
		},
		MapRateSec:    &mapRateSec,
		DataDirectory: dataDir,
		UseLiveData:   &useLiveData,
	}

	// Release point cloud for service validation
	testhelper.IntegrationLidarReleasePointCloudChan <- 1
	// Create slam service using a real cartographer binary
	svc, err = testhelper.CreateSLAMService(t, attrCfg, golog.NewTestLogger(t), true, viamcartographer.DefaultExecutableName)
	test.That(t, err, test.ShouldBeNil)

	// Make sure we initialize in updating mode
	logReader = svc.(testhelper.Service).GetSLAMProcessBufferedLogReader()
	for {
		line, err := logReader.ReadString('\n')
		test.That(t, err, test.ShouldBeNil)
		if strings.Contains(line, "Running in updating mode") {
			break
		}
		test.That(t, strings.Contains(line, "Running in localization only mode"), test.ShouldBeFalse)
		test.That(t, strings.Contains(line, "Running in mapping mode"), test.ShouldBeFalse)
	}

	// Release point cloud, since cartographer looks for the second most recent point cloud
	testhelper.IntegrationLidarReleasePointCloudChan <- 1
	for i := 0; i < testhelper.NumPointClouds-2; i++ {
		t.Logf("Find log line for point cloud %v", i)
		testhelper.IntegrationLidarReleasePointCloudChan <- 1
		for {
			line, err := logReader.ReadString('\n')
			test.That(t, err, test.ShouldBeNil)
			if strings.Contains(line, "Passed sensor data to SLAM") {
				prevNumFiles = testhelper.CheckDeleteProcessedData(
					t,
					subAlgo,
					dataDir,
					prevNumFiles,
					deleteProcessedData,
					useLiveData)
				break
			}
			test.That(t, strings.Contains(line, "Failed to open proto stream"), test.ShouldBeFalse)
			test.That(t, strings.Contains(line, "Failed to read SerializationHeader"), test.ShouldBeFalse)
		}
	}

	testCartographerPosition(t, svc, attrCfg.Sensors[0])
	testCartographerMap(t, svc, false)

	// Close out slam service
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

	// Test that a new map was generated
	testCartographerDir(t, dataDir, 2)

	// Clear out directory
	testhelper.ClearDirectory(t, dataDir)
}

// Checks the current slam directory to see if the number of files matches the expected amount.
func testCartographerDir(t *testing.T, path string, expectedMaps int) {
	mapsInDir, err := os.ReadDir(path + "/map/")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(mapsInDir), test.ShouldBeGreaterThanOrEqualTo, expectedMaps)
}

func TestCartographerIntegration2D(t *testing.T) {
	integrationtestHelperCartographer(t, viamcartographer.Dim2d)
}
