# 復号システム（decrypt_server）

暗号文字列の復号を担う独立サブシステム．
ラズパイ側とPC（無線通信デバイス）側の**どちらにも同一バイナリをデプロイでき**，
配置による性能差（遅延・演算能力）を実測比較するためのベンチマークを備える．

## 構成

| 実行ファイル | 役割 | 実行場所 |
|---|---|---|
| `decrypt_app` | 復号リクエストを待ち受けるサーバー | ラズパイ／PCの両方 |
| `decrypt_bench` | 3モードの性能を計測してCSVへ出力する | ラズパイ（機体側） |

| ディレクトリ | 内容 |
|---|---|
| `modules/utils/` | `Decrypter`（OpenSSLによるAES-128復号） |
| `modules/actions/` | `DecryptActionHandler`（復号実行と演算時間の計測） |
| `modules/server/` | `DecryptSocketServer`（接続待ち受けとコマンド分岐） |
| `modules/service/` | `LocalDecryptService`（プロセス内復号） |
| `modules/benchmark/` | 試行ループ・統計量算出・CSV出力 |

クライアント側（`DecryptClient` / `RemoteDecryptService`）は走行システムからも使えるよう
`modules/client/` と `modules/decrypt/` に置いてある．

> **重要**: `modules/` と `common/` 配下の `*.cpp` は `Makefile.inc` により
> RasPike-ART の走行アプリへ全て取り込まれる．したがってこれらに OpenSSL 依存のコードを
> 置いてはならない．OpenSSL 依存は `decrypt_server/` 配下に隔離すること．

## 計測する3モード

| モード | 復号の実行場所 | 通信 | 対応するトレードオフ案 |
|---|---|---|---|
| `LOCAL_INPROC` | ベンチプロセス内 | なし | （通信ゼロの基準値） |
| `LOCAL_SOCKET` | ラズパイ上の `decrypt_app` | loopback | 案B: Raspberry Piへ配置 |
| `REMOTE` | PC上の `decrypt_app` | 無線（Wi-Fi／Bluetooth PAN） | 案A: PCへ配置 |

計測値の内訳:

- `totalMicro` … クライアント側で計測した送信〜レスポンス受信完了までの時間
- `serverDecryptMicro` … 復号を実行した側で計測した，通信を含まない純粋な演算時間
- `commMicro` … `totalMicro - serverDecryptMicro`（通信＋シリアライズ＋サーバー内処理）
- `pingMicro` … 復号を行わない `PING` コマンドの往復時間（通信遅延のベースライン）

## プロトコル

定義は `common/DecryptProtocol.h`．ポート番号は **27016**（撮影システムは 27015）．

- POD構造体をそのまま送受信する．先頭1バイトが必ず `Command`．
- **バイトオーダ変換は行わない．** ラズパイ(aarch64)とPC(x86_64)が
  ともにリトルエンディアンのLP64環境であることを前提とする．
  構造体レイアウトの一致は `static_assert` で担保している．
- TCPのセグメント分割に備え，送受信は必ず `common/SocketIo.h` の
  `recvExact` / `sendAll` で全長を処理する．

## 使い方

PC側でサーバーを起動する（27016/tcp の受信をファイアウォールで許可しておくこと）。

```bash
./decrypt_app
```

ラズパイ側でサーバーを常駐させ、ベンチマークを実行する。

```bash
./decrypt_app &
```

```bash
./decrypt_bench --trials 100 --iterations 1 --remote-ip <PCのIPアドレス>
```

主なオプション（`--help` で一覧表示）:

| オプション | 既定値 | 内容 |
|---|---|---|
| `--trials` | 100 | 計測する試行回数 |
| `--warmup` | 10 | 計測前に捨てる試行回数 |
| `--iterations` | 1 | 1リクエストあたりの復号回数 |
| `--remote-ip` | （未指定） | PC側サーバーのIP．未指定なら`REMOTE`をスキップ |
| `--port` | 27016 | 復号サーバーのポート番号 |
| `--out` | `datafiles/logfiles/decrypt_benchmark.csv` | CSVの出力先 |

## 計測時の注意

- **ウォームアップを必ず行う．** 初回はOpenSSLの遅延初期化・ページフォルト・
  TCPスロースタート・無線リンクの省電力復帰が乗り、外れ値になる．
- **ラズパイのCPU周波数スケーリングとサーマルスロットリング**が結果を歪める．
  測定中はgovernorを`performance`に固定し、下記で温度とスロットリング状態を記録すること．

```bash
vcgencmd measure_temp && vcgencmd get_throttled
```

- ラズパイとPCで **g++のバージョンと最適化フラグを揃える**こと．
  **x86のAES-NIとARMv8 Crypto Extensionsの有無**が「演算能力」差の主因になりうるため、
  レポートには必ず記載する．
- 無線区間は分散が大きい．平均だけでなく**中央値と最大値（テールレイテンシ）**を併記する．
  ロボット制御では最大値が支配的になる．**Bluetooth PANはWi-Fiより実効帯域が低く遅延も
  大きくなりやすいため、通信方式が異なると`commMicro`の絶対値は単純比較できない．
  レポートには実測時の無線方式（Wi-Fi／Bluetooth PAN等）を明記すること．**
- `--iterations` を 1 / 10 / 100 と振ると、演算支配域と通信支配域の交点が観測でき、
  トレードオフ分析の説得力が上がる．
- 出力CSVは `datafiles/logfiles/` 配下で `.gitignore` 済み．
  レポート用に使う場合は手動で退避すること．

## セキュリティ上の注記

案A（PC側で復号）は**復号キーと暗号文を平文のまま無線で送出する**ため、
盗聴により鍵が漏洩する．競技会場の共用Wi-Fi環境や、Bluetooth PANのようなペアリング済み
デバイス間の接続であっても、この平文送出という性質自体は変わらない．
これは実運用上の欠点であり、性能効率性とは別軸（セキュリティ）のデメリットとして評価に含めること．
案B（ラズパイ内で復号）は鍵が機体外へ出ない．
