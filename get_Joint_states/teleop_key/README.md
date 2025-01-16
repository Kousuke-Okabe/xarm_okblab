#teleop_key
ROSを用いてキーボードで指令を送る。
Pythonでかかれたソースコードが多いが、本コードはc++である。

## 使用方法
```bash
$ rosrun teleop_key teleop_key
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.

```

`teleop_key.cpp`の最初の方で、キーコードの16進数を定義している。  
キー入力による条件分岐はswich文で行われている。  
実際にどんな値が出ているかは

```bash
$ rostopic echo /cmd_vel
linear: 
  x: 0.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
...
```

で確認できる。


## その他
ノード
* teleop_key

トピック
* cmd_vel