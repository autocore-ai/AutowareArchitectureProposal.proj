ARG IMG_SRC
ARG IMG_SDK
FROM ${IMG_SRC} as src
FROM ${IMG_SDK} as sdk
COPY --from=src /AutowareArchitectureProposal /AutowareArchitectureProposal
RUN cd /AutowareArchitectureProposal && \
    source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --catkin-skip-building-tests \
    --packages-up-to \
    sdv_demo_launch

FROM alpine
ARG REPO
LABEL org.opencontainers.image.source ${REPO}
COPY --from=sdk /AutowareArchitectureProposal/install /AutowareArchitectureProposal/install
COPY --from=sdk /AutowareArchitectureProposal.ref /AutowareArchitectureProposal/ref
COPY --from=sdk /AutowareArchitectureProposal.sha /AutowareArchitectureProposal/sha
