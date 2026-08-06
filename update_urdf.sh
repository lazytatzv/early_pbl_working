#!/bin/bash

# =============================================================================
# Onshape to URDF Export Script (URL Support Version)
# =============================================================================

# API Keys
export ONSHAPE_API="https://cad.onshape.com"
export ONSHAPE_ACCESS_KEY="on_rtqzeT9b2pDhSAvgNfrCd"
export ONSHAPE_SECRET_KEY="h2yWBLeKUFxUdKVDdYQHNuSIeD8qHdCJTca3nPW8bwoI0O8C"

ROBOT_DIR="onshape_robot"
CONFIG_FILE="$ROBOT_DIR/config.json"
mkdir -p "$ROBOT_DIR"

# URLが引数として渡された場合、IDを抽出してconfig.jsonを更新する
if [ ! -z "$1" ]; then
    URL="$1"
    echo "Extracting IDs from URL: $URL"
    
    # URLから各IDを抽出 (より堅牢な正規表現に変更)
    DOC_ID=$(echo $URL | sed -n 's/.*\/documents\/\([^\/]*\).*/\1/p')
    WKS_ID=$(echo $URL | sed -n 's/.*\/w\/\([^\/]*\).*/\1/p')
    ELE_ID=$(echo $URL | sed -n 's/.*\/e\/\([^\/]*\).*/\1/p')

    if [ -z "$DOC_ID" ] || [ -z "$WKS_ID" ] || [ -z "$ELE_ID" ]; then
        echo "Error: Could not extract IDs from URL. Make sure it's a valid Onshape assembly URL."
        exit 1
    fi

    # config.json を生成/更新
    cat <<EOF > "$CONFIG_FILE"
{
    "documentId": "$DOC_ID",
    "workspaceId": "$WKS_ID",
    "elementId": "$ELE_ID",
    "outputFormat": "urdf",
    "drawFrames": false,
    "drawCollisions": false,
    "useFixedLinks": true,
    "addDummyBaseLink": true
}
EOF
    echo "Config updated with new IDs."
fi

# Check if config exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: $CONFIG_FILE not found and no URL provided."
    echo "Usage: ./update_urdf.sh [ONSHAPE_URL]"
    exit 1
fi

echo "Starting Onshape-to-Robot export..."

# システムのPython環境エラーを避けるため、uvがあればそちらを優先する
if command -v uv > /dev/null && [ -d ".venv" ]; then
    echo "Using uv environment for stability..."
    uv run python -m onshape_to_robot.export "$ROBOT_DIR"
else
    onshape-to-robot "$ROBOT_DIR"
fi

if [ $? -eq 0 ]; then
    echo "Success! URDF and STL files are in $ROBOT_DIR/"
    
    # 生成されたURDFからパラメータを抽出してC++ヘッダーを自動生成
    echo "Generating include/UrdfConfig.h from robot.urdf..."
    if command -v uv > /dev/null && [ -d ".venv" ]; then
        uv run python scripts/parse_urdf.py
    else
        python scripts/parse_urdf.py
    fi
else
    echo "Error: Export failed."
    exit 1
fi
