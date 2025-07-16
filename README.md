# リポジトリの説明
ros2 jazzyの開発環境を構築をするためのレポジトリです

ip addr show eth0でipアドレスを調べる
netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=[WSL2のIPアドレス]でポート開ける。
netsh interface portproxy delete v4tov4 listenport=8765 listenaddress=0.0.0.0でポート削除
