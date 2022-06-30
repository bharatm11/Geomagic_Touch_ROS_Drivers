# bilateral

## Controller Design
- install requirements: MATLAB
  - or use [MATLAB online](https://matlab.mathworks.com/)

1. `$ ./scripts/calc_controller.sh`
2. Patch coefficients of **Hc** to **bilateral.hpp**
    - e.g.
        ```
        Hc =

          1121 z - 1115
          -------------
           z - 0.9876
        ```
        - a0: 1121
        - a1: -1115
        - b1: 0.9876
    - cf. [IIRフィルタの伝達関数設計](https://www.cqpub.co.jp/interface/sample/200609/I0609121.pdf)
3. build, source
4. `$ roslaunch bilateral dual.launch`
