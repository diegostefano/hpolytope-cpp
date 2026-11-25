FROM ubuntu:24.04

RUN set -ex; \
    apt update; \
    apt install -y g++ cmake libcdd-dev libeigen3-dev libgtest-dev;

WORKDIR /app

COPY . .

RUN set -ex; \
    cmake -S . -B build; \
    cmake --build build;

CMD ["ctest", "--test-dir", "build", "--output-on-failure"]