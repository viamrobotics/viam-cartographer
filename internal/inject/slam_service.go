// Package inject is used to mock a slam grpc client. This will be removed once the slam service is fully modularized and said
// grpc client disappears. JIRA Ticket: https://viam.atlassian.net/browse/RSDK-2730
package inject

import (
	"context"
	"errors"

	commonv1 "go.viam.com/api/common/v1"
	v1 "go.viam.com/api/service/slam/v1"
	"google.golang.org/grpc"
)

// SLAMServiceClient represents a fake instance of the client the slam service uses to communicate with the underlying SLAM algorithm.
type SLAMServiceClient struct {
	GetPositionFunc func(ctx context.Context, in *v1.GetPositionRequest, opts ...grpc.CallOption) (
		*v1.GetPositionResponse, error)
	GetPointCloudMapFunc func(ctx context.Context, in *v1.GetPointCloudMapRequest, opts ...grpc.CallOption) (
		v1.SLAMService_GetPointCloudMapClient, error)
	GetInternalStateFunc func(ctx context.Context, in *v1.GetInternalStateRequest, opts ...grpc.CallOption) (
		v1.SLAMService_GetInternalStateClient, error)
	GetLatestMapInfoFunc func(ctx context.Context, in *v1.GetLatestMapInfoRequest, opts ...grpc.CallOption) (
		*v1.GetLatestMapInfoResponse, error)
	DoCommandFunc func(ctx context.Context, in *commonv1.DoCommandRequest, opts ...grpc.CallOption) (
		*commonv1.DoCommandResponse, error)
}

// GetPosition calls the injected GetPositionFunc if defined else it will return an error.
func (slamSvcClient *SLAMServiceClient) GetPosition(ctx context.Context, in *v1.GetPositionRequest, opts ...grpc.CallOption) (
	*v1.GetPositionResponse, error,
) {
	if slamSvcClient.GetPositionFunc == nil {
		return nil, errors.New("no GetPositionFunc defined for injected SLAM service client")
	}
	return slamSvcClient.GetPositionFunc(ctx, in, opts...)
}

// GetPointCloudMap calls the injected GetPointCloudMap if defined else it will return an error.
func (slamSvcClient *SLAMServiceClient) GetPointCloudMap(ctx context.Context, in *v1.GetPointCloudMapRequest, opts ...grpc.CallOption) (
	v1.SLAMService_GetPointCloudMapClient, error,
) {
	if slamSvcClient.GetPointCloudMapFunc == nil {
		return nil, errors.New("no GetPointCloudMapFunc defined for injected SLAM service client")
	}
	return slamSvcClient.GetPointCloudMapFunc(ctx, in, opts...)
}

// GetInternalState calls the injected GetInternalState if defined else it will return an error.
func (slamSvcClient *SLAMServiceClient) GetInternalState(ctx context.Context, in *v1.GetInternalStateRequest, opts ...grpc.CallOption) (
	v1.SLAMService_GetInternalStateClient, error,
) {
	if slamSvcClient.GetInternalStateFunc == nil {
		return nil, errors.New("no GetInternalState defined for injected SLAM service client")
	}
	return slamSvcClient.GetInternalStateFunc(ctx, in, opts...)
}

// GetLatestMapInfo calls the injected GetLatestMapInfo if defined else it will return an error.
func (slamSvcClient *SLAMServiceClient) GetLatestMapInfo(ctx context.Context, in *v1.GetLatestMapInfoRequest, opts ...grpc.CallOption) (
	*v1.GetLatestMapInfoResponse, error,
) {
	if slamSvcClient.GetLatestMapInfoFunc == nil {
		return nil, errors.New("no GetLatestMapInfoFunc defined for injected SLAM service client")
	}
	return slamSvcClient.GetLatestMapInfoFunc(ctx, in, opts...)
}

// DoCommand calls the injected DoCommand if defined else it will return an error.
func (slamSvcClient *SLAMServiceClient) DoCommand(ctx context.Context, in *commonv1.DoCommandRequest, opts ...grpc.CallOption) (
	*commonv1.DoCommandResponse, error,
) {
	if slamSvcClient.DoCommandFunc == nil {
		return nil, errors.New("no DoCommand defined for injected SLAM service client")
	}
	return slamSvcClient.DoCommandFunc(ctx, in, opts...)
}
