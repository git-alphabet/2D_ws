import re

with open('scripts/launch_wrapper.py', 'r') as f:
    text = f.read()

# Make _runtime_log_dir use BEIJING_TZ timestamp
def replace_log_dir(match):
    return """def _runtime_log_dir(cfg) -> Path:
    branch = _current_branch(cfg.ws_dir)
    branch_safe = re.sub(r'[^A-Za-z0-9._-]', '_', branch)
    ts = _beijing_timestamp()
    d = cfg.ws_dir / RUNTIME_LOG_DIR_NAME / branch_safe / f"log_{ts}_{cfg.script_base}"
    d.mkdir(parents=True, exist_ok=True)
    return d"""

text = re.sub(
    r'def _runtime_log_dir\(cfg.*?return d',
    replace_log_dir,
    text,
    flags=re.DOTALL
)

with open('scripts/launch_wrapper.py', 'w') as f:
    f.write(text)
