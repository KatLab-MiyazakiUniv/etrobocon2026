MAKEFILE_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

DOCKER_IMAGE   := kat_etrobo2026:arm64
DOCKER_WORKDIR := /RasPike-ART/sdk/workspace/etrobocon2026
DOCKER_MOUNT   := -v $(MAKEFILE_PATH):$(DOCKER_WORKDIR)

# 全システムの使い方
help:
	@echo 全システムをビルドする
	@echo " $$ make build"
	@echo 走行システムをビルドする
	@echo " $$ make build-running"
	@echo 撮影システムをビルドする
	@echo " $$ make build-camera"
	@echo 走行を開始する\(実機限定\)
	@echo " $$ make start"
	@echo 走行システムを開始する
	@echo " $$ make start-running"
	@echo 撮影システムを開始する
	@echo " $$ make start-camera"
	@echo 指定ファイルをフォーマットする
	@echo " $$ make format FILES=<ディレクトリ名>/<ファイル名>.cpp"
	@echo すべての変更ファイルをフォーマットする
	@echo " $$ make format"
	@echo フォーマットチェックをする
	@echo " $$ make format-check"
	@echo Motions/Conditions以下の全コマンドCSVを型チェックする
	@echo " $$ make check-type-commands"
	@echo 全エリアのArea CSVを型チェックする
	@echo " $$ make check-type-areas"
	@echo 指定ファイルのArea CSVを型チェックする
	@echo " $$ make check-type FILE=<ファイル名.csv>"
	@echo 全システムのテストをビルドする
	@echo " $$ make test-build"
	@echo 走行システムのテストをビルドする
	@echo " $$ make test-build-running"
	@echo 撮影システムのテストをビルドする
	@echo " $$ make test-build-camera"
	@echo 全システムのテストを実行する
	@echo " $$ make test-exec"
	@echo 走行システムのテストを実行する
	@echo " $$ make test-exec-running"
	@echo 撮影システムのテストを実行する
	@echo " $$ make test-exec-camera"
	@echo 全システムのテストをビルドして実行する
	@echo " $$ make test"
	@echo 走行システムのテストをビルドして実行する
	@echo " $$ make test-running"
	@echo 撮影システムのテストをビルドして実行する
	@echo " $$ make test-camera"
	@echo テスト用の'test_build'ディレクトリを削除する
	@echo " $$ make clean"
	@echo 環境が変わっている場合のみ test_build ディレクトリを削除する
	@echo " $$ make smart-clean"
	@echo Dockerイメージをビルドする
	@echo " $$ make docker-build"
	@echo Dockerコンテナを起動する
	@echo " $$ make docker-run"
	@echo UID/GIDを指定してDockerコンテナを起動する\(権限問題が起きた場合\)
	@echo " $$ make docker-run-user"

## 実行関連 ##
.PHONY: build
build: build-running build-camera

# 走行システムをビルドする
build-running:
	cd $(MAKEFILE_PATH)../ && make img=etrobocon2026 -j5

# 撮影システムをビルドする
build-camera:
	cd $(MAKEFILE_PATH)camera_server && make -f Makefile.camera -j5

# 実機の場合、走行を開始する
start: start-camera start-running

# 走行システムを開始する
start-running:
	cd $(MAKEFILE_PATH)../ && make start

# 撮影システムを開始する
start-camera:
	cd $(MAKEFILE_PATH)camera_server && ./camera_app

## 開発関連 ##
FORMAT_FILES := find . \
	-path "./test_build" -prune -o \
	-path "./build" -prune -o \
	-path "./.git" -prune -o \
	-type f \( -name "*.cpp" -o -name "*.h" \) -print

# ファイルにclang-formatを適用する
format:
ifdef FILES
	clang-format -i -style=file $(FILES)
	@echo "フォーマットを適用しました: $(FILES)"
else
	$(FORMAT_FILES) | xargs clang-format -i -style=file
	@echo "すべての .cpp / .h ファイルにフォーマットを適用しました。"
endif

format-check:
	$(FORMAT_FILES) | xargs clang-format --dry-run --Werror -style=file

# Motions/ と Conditions/ 以下の全コマンドCSVを型チェックする
check-type-commands:
	./scripts/check_type.sh --commands

# datafiles/commands/Area/ 以下の全 Area CSV を自動探索して型チェックする
check-type-areas:
	@set -e; for f in $$(find datafiles/commands/Area -name "*.csv" | sort); do \
		./scripts/check_type.sh "$$f"; \
	done

# 指定ファイルの Area CSV を型チェックする（例: make check-type FILE=LineTraceLeft.csv）
check-type:
	./scripts/check_type.sh datafiles/commands/Area/$(FILE)

## テスト関連 ##
# 全システムのテストをビルドする
test-build:
	@mkdir -p $(MAKEFILE_PATH)test_build
	cd $(MAKEFILE_PATH)test_build && cmake .. && make

# 走行システムのテストをビルドする
test-build-running:
	@mkdir -p $(MAKEFILE_PATH)test_build
	cd $(MAKEFILE_PATH)test_build && cmake .. -DENABLE_CAMERA_TESTS=OFF && make running_test

# 撮影システムのテストをビルドする
test-build-camera:
	@mkdir -p $(MAKEFILE_PATH)test_build
	cd $(MAKEFILE_PATH)test_build && cmake .. -DENABLE_CAMERA_TESTS=ON && make camera_server_test

# 全システムのテストを実行する
test-exec: test-exec-running test-exec-camera

# 走行システムのテストを実行する
test-exec-running:
	@if [ ! -f $(MAKEFILE_PATH)test_build/running_test ]; then \
		echo "テスト実行ファイルが見つかりません。まずビルドを実行してください。"; \
		echo " $$ make test-build-running"; \
		exit 1; \
	fi
	cd $(MAKEFILE_PATH)test_build && ./running_test

# 撮影システムのテストを実行する
test-exec-camera:
	@if [ ! -f $(MAKEFILE_PATH)test_build/camera_server_test ]; then \
		echo "テスト実行ファイルが見つかりません。まずビルドを実行してください。"; \
		echo " $$ make test-build-camera"; \
		exit 1; \
	fi
	cd $(MAKEFILE_PATH)test_build && ./camera_server_test

# 全システムのテストをビルドして実行する
test: smart-clean test-build test-exec

# 走行システムのテストをビルドして実行する
test-running: smart-clean test-build-running test-exec-running

# 撮影システムのテストをビルドして実行する
test-camera: smart-clean test-build-camera test-exec-camera

# test_build ディレクトリを完全に削除する
clean:
	@if [ -d $(MAKEFILE_PATH)test_build ]; then \
		rm -rf $(MAKEFILE_PATH)test_build; \
		echo "'test_build/' ディレクトリを削除しました。"; \
	else \
		echo "'test_build/' ディレクトリは既に存在しません。"; \
	fi

# 実行環境が変更されている場合にのみ 'test_build' を削除する
smart-clean:
	@if [ -d $(MAKEFILE_PATH)test_build ]; then \
		if [ -f "$(MAKEFILE_PATH)test_build/Makefile" ]; then \
			CMAKE_SOURCE_DIR=`grep -E "^CMAKE_SOURCE_DIR[[:space:]]*=" $(MAKEFILE_PATH)test_build/Makefile | cut -d= -f2 | xargs`; \
			CMAKE_SOURCE_DIR_REAL=`readlink -f "$$CMAKE_SOURCE_DIR"`; \
			CURRENT_DIR_REAL=`readlink -f "$$(pwd)"`; \
			echo "[DEBUG] CMAKE_SOURCE_DIR: '$$CMAKE_SOURCE_DIR_REAL'"; \
			echo "[DEBUG] CURRENT_DIR    : '$$CURRENT_DIR_REAL'"; \
			if [ "$$CMAKE_SOURCE_DIR_REAL" != "$$CURRENT_DIR_REAL" ]; then \
				echo "[LOG] 実行環境の変更が検出されたため 'test_build/' を削除します。"; \
				rm -rf $(MAKEFILE_PATH)test_build; \
			else \
				echo "[LOG] 実行環境は変更されていません。"; \
			fi; \
		fi; \
	else \
		echo "'test_build/' ディレクトリは既に存在しません。"; \
	fi

## Docker関連 ##
docker-build:
	docker buildx build --platform linux/arm64 -t $(DOCKER_IMAGE) .

docker-run:
	docker run -it --rm \
		$(DOCKER_MOUNT) \
		$(DOCKER_IMAGE) bash

docker-run-user:
	docker run -it --rm \
		--user $$(id -u):$$(id -g) \
		$(DOCKER_MOUNT) \
		$(DOCKER_IMAGE) bash