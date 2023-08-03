// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"math"
	"strings"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/components/camera/replaypcd"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors"
)

// Config holds config needed throughout the process of adding a sensor reading to the cartofacade.
type Config struct {
	CartoFacade       cartofacade.Interface
	Lidar             sensors.TimedSensor
	LidarName         string
	LidarDataRateMsec int
	Timeout           time.Duration
	Logger            golog.Logger
}

// Start polls the lidar to get the next sensor reading and adds it to the cartofacade.
// stops when the context is Done.
func Start(
	ctx context.Context,
	config Config,
	isOnline bool,
) bool {
	for {
		select {
		case <-ctx.Done():
			return false
		default:
			if jobDone := addSensorReading(ctx, config, isOnline); jobDone {
				return true
			}
		}
	}
}

// addSensorReading adds a lidar reading to the cartofacade.
func addSensorReading(
	ctx context.Context,
	config Config,
	isOnline bool,
) bool {
	tsr, err := config.Lidar.TimedSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		return strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error())
	}
	if isOnline {
		timeToSleep := addSensorReadingOnline(ctx, tsr.Reading, tsr.ReadingTime, config)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	} else {
		addSensorReadingOffline(ctx, tsr.Reading, tsr.ReadingTime, config)
	}
	return false
}

// addSensorReadingOffline adds a reading to the cartofacade
// retries on error.
func addSensorReadingOffline(ctx context.Context, reading []byte, readingTime time.Time, config Config) {
	/*
		while add sensor reading fails, keep trying to add the same reading - in offline mode
		we want to process each reading so if we cannot acquire the lock we should try again
	*/
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddSensorReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
			if err == nil {
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
			}
		}
	}
}

// addSensorReadingOnline adds a reading to the carto facade
// does not retry.
func addSensorReadingOnline(ctx context.Context, reading []byte, readingTime time.Time, config Config) int {
	startTime := time.Now()
	err := config.CartoFacade.AddSensorReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
	if err != nil {
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping sensor reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
		}
	}
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(config.LidarDataRateMsec-timeElapsedMs)))
}
