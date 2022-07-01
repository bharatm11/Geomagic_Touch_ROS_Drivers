#!/bin/bash

cd $(rospack find bilateral)/scripts
# matlabをコマンドラインで動かす
# コマンドはcalc_controller.mを参照
matlab -nodisplay -nosplash -nodesktop -r "calc_controller; quit"
