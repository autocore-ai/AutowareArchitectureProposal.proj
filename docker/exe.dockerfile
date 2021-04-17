ARG IMG_SRC
ARG IMG_SDK
FROM ${IMG_SRC} as src
FROM ${IMG_SDK} as sdk
COPY --from=src /AutowareArchitectureProposal /AutowareArchitectureProposal
RUN cd /AutowareArchitectureProposal && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --catkin-skip-building-tests \
    --packages-up-to \
    planning_simulator_launcher

FROM alpine
ARG REPO
LABEL org.opencontainers.image.source ${REPO}
COPY --from=sdk /AutowareArchitectureProposal/install /AutowareArchitectureProposal/install
COPY --from=src /AutowareArchitectureProposal.ref /AutowareArchitectureProposal.ref
COPY --from=src /AutowareArchitectureProposal.ref /AutowareArchitectureProposal/ref
COPY --from=src /AutowareArchitectureProposal.sha /AutowareArchitectureProposal.sha
COPY --from=src /AutowareArchitectureProposal.sha /AutowareArchitectureProposal/sha
