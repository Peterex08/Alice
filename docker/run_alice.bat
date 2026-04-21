@echo off
REM Define o display para o X Server rodando no Windows (ex: VcXsrv)
set DISPLAY=host.docker.internal:0.0

echo Iniciando o container Alice...

docker run -it --rm ^
    --name alice ^
    --env="DISPLAY=%DISPLAY%" ^
    --env="QT_X11_NO_MITSHM=1" ^
    --env="HOST_USERNAME=%USERNAME%" ^
    --volume="%cd%\..:/home/rosuser/Alice:rw" ^
    --privileged ^
    --gpus all ^
    --env="NVIDIA_DRIVER_CAPABILITIES=all" ^
    --env="NVIDIA_VISIBLE_DEVICES=all" ^
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia ^
    -e __NV_PRIME_RENDER_OFFLOAD=1 ^
    -e __VK_LAYER_NV_optimus=NVIDIA_only ^
    alice