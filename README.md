## installing

uv venv
git clone https://github.com/huggingface/lerobot.git
cd lerobot
uv pip install .[feetech]

## ffmpeg 8 not supported by lerobot yet

this snippet fixes that on macOS with uv

brew install ffmpeg@7
export PATH="/opt/homebrew/opt/ffmpeg@7/bin:$PATH"
