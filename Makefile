MAKEFILE_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# 使い方
help:
	@echo ビルドする
	@echo " $$ make build"
	@echo 走行を開始する\(実機限定\)
	@echo " $$ make start"
	@echo 指定ファイルをフォーマットする
	@echo " $$ make format FILES=<ディレクトリ名>/<ファイル名>.cpp"
	@echo すべての変更ファイルをフォーマットする
	@echo " $$ make format"
	@echo フォーマットチェックをする
	@echo " $$ make format-check"
	@echo テストをビルドする
	@echo " $$ make test-build"
	@echo テストを実行する
	@echo " $$ make test-exec"
	@echo テスト用の'test_build'ディレクトリを削除する
	@echo " $$ make clean"
	@echo 環境が変わっている場合のみ test_build ディレクトリを削除する
	@echo " $$ make smart-clean"
	@echo 必要があればtest_buildディレクトリを削除し, テストをビルドして実行する
	@echo " $$ make test"

## 実行関連 ##
.PHONY: build
build:
	cd $(MAKEFILE_PATH)../ && make img=etrobocon2026

# 実機の場合、走行を開始する
start:
	cd $(MAKEFILE_PATH)../ && make start

## テスト関連 ##
# テストのビルドディレクトリが存在しない場合は作成する
test-build:
	@mkdir -p $(MAKEFILE_PATH)test_build
	cd $(MAKEFILE_PATH)test_build && cmake .. && make

# テストを実行する
test-exec:
	@if [ ! -f $(MAKEFILE_PATH)test_build/etrobocon2026_test ]; then \
		echo "テスト実行ファイルが見つかりません。まずビルドを実行してください。"; \
		echo " $$ make test-build"; \
		exit 1; \
	fi
	cd $(MAKEFILE_PATH)test_build && ./etrobocon2026_test

# テストをビルドして実行する
test: smart-clean test-build test-exec

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