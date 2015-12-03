#!/bin/bash

blue='\e[0;34m'
NC='\e[0m' # No Color
red='\e[0;31m'
green='\e[0;32m' 
 
case "$1" in
    start)
        echo "Iniciando serviço de imu"
	gnome-terminal -x sh -c 'imu'
        echo "Iniciando serviço de vision"
	gnome-terminal -x sh -c 'vision'
        echo "Iniciando serviço de decision"
	gnome-terminal -x sh -c 'decision'
        echo "Iniciando serviço de control"
	gnome-terminal -x sh -c 'control'
        ;;
    *)
        echo "Operação inválida"
        ;;
esac
