# gps-mapmatching
GPS 데이터의 튀는 현상을 보정하기 위해 Leuven map matching 패키지를 이용하여 보정하였습니다.

사용한 도로 네트워크 데이터는 open street map입니다.

## 개발환경
Ubuntu 18.04, python3

## pip install
leuvenmapmatching

smopy

geopandas

rdp

## conda install 
osmnx

## yaml 파일로 가상환경 생성 (아나콘다 설치 후)
conda env create -f gps_venv.yaml
