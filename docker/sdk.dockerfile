FROM ros:foxy
ARG REPO
ARG REF
ARG SHA
LABEL org.opencontainers.image.source ${REPO}
RUN echo ${REF} > /AutowareArchitectureProposal.ref
RUN echo ${SHA} > /AutowareArchitectureProposal.sha
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
COPY docker/rosdep.sh /tmp/rosdep.sh
RUN apt-get update \
    && sh /tmp/rosdep.sh \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install gdown
COPY docker/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
