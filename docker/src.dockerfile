FROM alpine
ARG REPO
LABEL org.opencontainers.image.source ${REPO}
COPY . /AutowareArchitectureProposal
