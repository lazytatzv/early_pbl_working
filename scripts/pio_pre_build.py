Import("env")
import os
import sys

project_dir = env.get("PROJECT_DIR")
sys.path.append(os.path.join(project_dir, "scripts"))
import generate_config
generate_config.main()
