#!/bin/zsh

mosquitto_sub -h mqtt.eclipseprojects.io -t nimbus/modular_sensor_platform | jq -rc '.pressure[0:2], .pressure[2:4], .presence[0:8], .presence[8:16], .presence[16:24], .presence[24:32], .presence[32:40], .presence[40:48], .presence[48:56], .presence[56:64] | @tsv'
