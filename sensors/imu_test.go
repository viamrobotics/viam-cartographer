// Package sensors_test implements tests for sensors
package sensors_test

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/test"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

func TestNewIMU(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("No IMU provided", func(t *testing.T) {
		imu := ""
		lidar := "good_lidar"
		deps := s.SetupDeps(lidar, imu)
		_, err := s.NewIMU(context.Background(), deps, imu, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Failed IMU creation with non-existing sensor", func(t *testing.T) {
		lidar := "good_lidar"
		imu := "gibberish"
		deps := s.SetupDeps(lidar, imu)
		actualIMU, err := s.NewIMU(context.Background(), deps, imu, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting IMU movement sensor "+
				"gibberish for slam service: \"rdk:component:movement_sensor/gibberish\" missing from dependencies"))
		expectedIMU := s.IMU{}
		test.That(t, actualIMU, test.ShouldResemble, expectedIMU)
	})

	t.Run("Failed IMU creation with sensor that does not support AngularVelocity", func(t *testing.T) {
		lidar := "good_lidar"
		imu := "imu_with_invalid_properties"
		deps := s.SetupDeps(lidar, imu)
		actualIMU, err := s.NewIMU(context.Background(), deps, imu, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring IMU movement sensor error: "+
				"'movement_sensor' must support both LinearAcceleration and AngularVelocity"))
		expectedIMU := s.IMU{}
		test.That(t, actualIMU, test.ShouldResemble, expectedIMU)
	})

	t.Run("Successful IMU creation", func(t *testing.T) {
		lidar := "good_lidar"
		imu := "good_imu"
		ctx := context.Background()
		deps := s.SetupDeps(lidar, imu)
		actualIMU, err := s.NewIMU(ctx, deps, imu, logger)
		test.That(t, actualIMU.Name, test.ShouldEqual, imu)
		test.That(t, err, test.ShouldBeNil)

		tsr, err := actualIMU.TimedIMUSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.LinearAcceleration, test.ShouldResemble, s.LinAcc)
		test.That(t, tsr.AngularVelocity, test.ShouldResemble,
			spatialmath.AngularVelocity{
				X: rdkutils.DegToRad(s.AngVel.X),
				Y: rdkutils.DegToRad(s.AngVel.Y),
				Z: rdkutils.DegToRad(s.AngVel.Z),
			})
	})
}

func TestTimedIMUSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	lidar := "good_lidar"
	imu := "imu_with_erroring_functions"
	imuWithErroringFunctions, err := s.NewIMU(ctx, s.SetupDeps(lidar, imu), imu, logger)
	test.That(t, err, test.ShouldBeNil)

	imu = "good_imu"
	goodIMU, err := s.NewIMU(ctx, s.SetupDeps(lidar, imu), imu, logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("when the IMU returns an error, returns that error", func(t *testing.T) {
		tsr, err := imuWithErroringFunctions.TimedIMUSensorReading(ctx)
		msg := "invalid sensor"
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
		test.That(t, tsr, test.ShouldResemble, s.TimedIMUSensorReadingResponse{})
	})

	t.Run("when a live IMU succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		beforeReading := time.Now().UTC()
		time.Sleep(time.Millisecond)

		tsr, err := goodIMU.TimedIMUSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.LinearAcceleration, test.ShouldResemble, s.LinAcc)
		test.That(t, tsr.AngularVelocity, test.ShouldResemble,
			spatialmath.AngularVelocity{
				X: rdkutils.DegToRad(s.AngVel.X),
				Y: rdkutils.DegToRad(s.AngVel.Y),
				Z: rdkutils.DegToRad(s.AngVel.Z),
			})
		test.That(t, tsr.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, tsr.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, tsr.Replay, test.ShouldBeFalse)
	})
}
