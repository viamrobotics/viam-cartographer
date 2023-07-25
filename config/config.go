// Package config implements functions to assist with attribute evaluation in the SLAM service.
package config

import (
	"strconv"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.viam.com/utils"
)

// newError returns an error specific to a failure in the SLAM config.
func newError(configError string) error {
	return errors.Errorf("SLAM Service configuration error: %s", configError)
}

// Config describes how to configure the SLAM service.
type Config struct {
	Camera        map[string]string `json:"camera"`
	ConfigParams  map[string]string `json:"config_params"`
	DataDirectory string            `json:"data_dir"`
	MapRateSec    *int              `json:"map_rate_sec"`
}

var errCameraMustHaveName = errors.New("\"camera[name]\" is required")

// Validate creates the list of implicit dependencies.
func (config *Config) Validate(path string) ([]string, error) {
	_, ok := config.Camera["name"]
	if !ok {
		return nil, utils.NewConfigValidationError(path, errCameraMustHaveName)
	}
	dataFreqHz, ok := config.Camera["data_freq_hz"]
	if ok {
		dataFreqHz, err := strconv.Atoi(dataFreqHz)
		if err != nil {
			return nil, errors.New("data_freq_hz must only contain digits")
		}
		if dataFreqHz < 0 {
			return nil, errors.New("cannot specify data_freq_hz less than zero")
		}
	}

	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	if config.DataDirectory == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "data_dir")
	}

	if config.MapRateSec != nil && *config.MapRateSec < 0 {
		return nil, errors.New("cannot specify map_rate_sec less than zero")
	}

	deps := []string{config.Camera["name"]}

	return deps, nil
}

// GetOptionalParameters sets any unset optional config parameters to the values passed to this function,
// and returns them.
func GetOptionalParameters(config *Config, defaultMapRateSec int, logger golog.Logger,
) int {
	mapRateSec := 0
	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_sec given, setting to default value of %d", defaultMapRateSec)
		mapRateSec = defaultMapRateSec
	} else {
		mapRateSec = *config.MapRateSec
	}

	return mapRateSec
}
