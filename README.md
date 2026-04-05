## Installing

```bash
uv venv
git clone https://github.com/huggingface/lerobot.git
cd lerobot
uv pip install .[feetech]
```

## ffmpeg 8 is not supported by LeRobot yet

This snippet fixes that on macOS with `uv`:

```bash
brew install ffmpeg@7
export PATH="/opt/homebrew/opt/ffmpeg@7/bin:$PATH"
```
