# bilateral

## 制御器設計
1. `$ ./scripts/calc_controller.sh`
2. terminalに表示されるHcをmaster/slave.cppに反映させる
    - 例:
        ```
        Hc =

          1121 z - 1115
          -------------
           z - 0.9876
        ```
        - a0: 1121
        - a1: -1115
        - b1: 0.9876
    - 参考: [IIRフィルタの伝達関数設計](https://www.cqpub.co.jp/interface/sample/200609/I0609121.pdf)
3. build, source
4. `$ roslaunch bilateral dual.launch`
