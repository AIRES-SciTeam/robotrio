rsync -av \
  --exclude='.venv/' \
  --exclude='__pycache__/' \
  --exclude='*.egg-info/' \
  --exclude='build/' \
  --exclude='config.json' \
  ~/robotrio/dog/onboard/ \
  pi@$1:~/robotrio_src/

rsync -av \
  ~/robotrio/dog/onboard/config.json \
  pi@$1:~/robotrio_ws/