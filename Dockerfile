FROM python:3.10-slim

# ffmpeg と pybullet/imageio 用の共有ライブラリをインストール
RUN apt-get update && apt-get install -y \
    ffmpeg \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Python依存パッケージを直接インストール (コンパイル不要なLinux-x86_64バイナリが降ります)
RUN pip install --no-cache-dir \
    pybullet==3.2.7 \
    numpy \
    pyyaml \
    imageio \
    imageio-ffmpeg

CMD ["python", "scripts/pybullet_simulation.py"]
