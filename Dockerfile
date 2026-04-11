# 実機環境(Debian Trixie AArch64)を構築するためのDockerfile
FROM --platform=linux/arm64 debian:trixie

# 必要なパッケージをインストール
RUN apt-get update && apt-get install -y \
  build-essential \
  ruby \
  git \
  cmake \
  clang-format \
  libopencv-dev \
  nlohmann-json3-dev \
  python3 \
  python3-venv \
  python3-pip \
  libusb-1.0-0-dev \
  && gem install shell \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

# ビルド時に日本語が含まれていたらエラーになるので，C.UTF-8を指定
ENV LANG=C.UTF-8

# RasPike-ART を clone
# libraspike-artは，RasPike-ART内で運営がリンクで紐づけていたため，直接cloneする必要がある
RUN git clone --depth=1 https://github.com/ETrobocon/RasPike-ART.git /RasPike-ART \
  && git clone --depth=1 https://github.com/ETrobocon/libraspike-art.git /RasPike-ART/libraspike-art \
  && cd /RasPike-ART/libraspike-art \
  && git submodule update --init ./external/

# 作業ディレクトリを/RasPike-ART/sdk/workspace/etrobocon2026に設定
WORKDIR /RasPike-ART/sdk/workspace/etrobocon2026
