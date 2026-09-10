# etrobocon2026
宮崎大学片山徹郎研究室チーム KatLab が作成する[ET ロボコン 2026](https://www.etrobo.jp/)アプライドクラスの走行システムプログラムです。

## 構成
### ./modules
ソースファイルを格納
### ./tests
テストソースを格納

## プロジェクトのビルド
### 実機の場合
```etrobocon2026```で以下を実行
```shell
make build
```
### 実機以外の場合
#### etrobo環境( 速いため，こちらを推奨 )
```~/etrobo/raspike-athrill-v850e2m/sdk/workspace/etrobocon2026```で以下を実行
```shell
make build
```

#### Docker仮想環境
- docker-composeを利用する場合（速いのでおすすめ）
```shell
docker-compose run --rm etrobocon
```

- Dockerイメージから構築する場合
```etrobocon2026```で以下を順に実行
1. (まだの場合は)trixie環境のイメージを以下のコマンドで作成
```shell
docker buildx build --platform linux/arm64 -t kat_etrobo2026:arm64 .
```
2. コンテナ作成と起動
```shell
docker run -it --rm -v $(pwd):/RasPike-ART/sdk/workspace/etrobocon2026 kat_etrobo2026:arm64 bash
```
もし，ファイル保存などの権限問題に引っかかったら，以下のコマンドでUIDとGIDを確認し，
```shell
id -u
```
```shell
id -g
```
その番号でそれぞれ，以下のコマンドのUIDとGIDを置き換えて実行してみてください
```shell
docker run -it --rm --user UID:GID -v $(pwd):/RasPike-ART/sdk/workspace/etrobocon2026 kat_etrobo2026:arm64 bash
```

3. プロジェクトのビルド
```shell
make build
```

## テスト
```etrobocon2026```で以下を実行
```shell
make test
```
もしくは，```etrobocon2026```で以下を実行
```shell
make smart-clean
make test-build
make test-exec
```

## ET相撲〜フィニッシュの調整

`Area/ETZumoLeft.csv` は複合動作 `ETZumoFinish,001` を実行します。
`datafiles/commands/Motions/ETZumoFinish.csv` の各4列（動作名、動作ID、条件名、条件ID）で順序を変更できます。自身の参照は禁止です。親条件には `RepeatCount,001` を指定してください。

現在の順序は、ET相撲の追従→回頭→前進→黒までバック→-75度回頭→カメラ追従100 mm→カメラ追従500 mm以上かつ青検知→-90度回頭→カメラ追従700 mm→停止です。

- `Motions/ETZumoExit.csv` の距離は、相撲開始時の向きを基準に、追従・回頭・前進を通して積算した投影距離です。現在は700 mm以上になってから黒を3回連続で検知すると最後の前進を終了します。任意の9列目 `arrivalColor` を省略すると従来の距離だけの終了条件になります。
- バックは `Conditions/DistanceAndColor.csv` の006（80 mm以上かつ黒）です。距離到達だけでは終了しません。
- 通常の動作開始位置から700 mm以上かつ黒にしたい場合は `DistanceAndColor,005` を使えます。
- バック後の追従は `CameraTracking,015` を使用します。速度、画面内の目標X、ROI、PIDは実コースで調整してください。
- 絶対回頭（`AbsoluteRotation`）の最低・最大パワーは `modules/motions/AbsoluteRotation.h` の18 / 60です。相対回頭には適用しません。最低値は搭載重量・電池状態を含めて実機で調整してください。
- センサーの色判定は暫定的にV<30を黒、30<=V<80かつS<37をGRAY（灰色）、同じ明度でS>=37をNONE（判定保留）、V>=80で彩度・色相判定としています。黒・灰色・青の実測HSVを集めて `ColorSensorController.h` の境界を調整してください。カメラ画像のHSV閾値とは別です。

黒や青を検出できないとAND条件の走行は終了しません。実機での検知と停止位置の確認が必要です。


### 複合動作内でのX・Y累積距離

`ProjectedMileage` は相撲開始時の方位をX（従来の水平成分）、その+90度方向をY（垂直成分）として、`Δ距離 × cos(方位)` と `Δ距離 × sin(方位)` を毎周期積算します。コース平面上の2軸であり、高さではありません。後退は減算され、走行経路の総延長ではなく起点からの符号付き変位を表します。

`ETZumoFinish` 内では相撲退出、バック、回頭、カメラ追従で同じ計測を共有します。途中で原点が変わるのを防ぐため、`ResetAzimuth` は指定できず、`ETZumoExit` は先頭にのみ指定できます。終了ログにはXとYの両方を出力します。

`Conditions/ProjectedDistance.csv` で軸と目標座標を指定できます。例えば複合動作CSVの子動作を `CameraTracking,015,ProjectedDistance,002` にすると、相撲開始地点からのYが-700 mm以下で終了します。001はX>=700 mm、003はY>=700 mmです。子動作開始時に距離はリセットされません。`ProjectedDistance` は `ETZumoFinish` 内専用です。

実コースのY目標値は未指定のため、現在の走行CSVの終了条件は維持しています。C++からは `getHorizontalDistance()` / `getVerticalDistance()` で取得でき、従来の `getDistance()` はXを返します。
