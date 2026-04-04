import re

with open('/home/alphabet/.config/Code/User/workspaceStorage/a0f287ddef8015e152b49bb3091ab8db/GitHub.copilot-chat/chat-session-resources/b498cae0-49ab-45e2-ba30-5e144930cf2e/call_MHxrVXI2NWlLU0JWWXNubXhtMko__vscode-1775292361049/content.txt', 'r') as f:
    lines = f.readlines()

out = []
for line in lines:
    line = line.rstrip('\r\n')
    if not line:
        continue
    # skip some meta diff headers
    if line.startswith('Note: The tool') or line.startswith('and this is'):
        continue
    if line.startswith('diff --git') or line.startswith('index ') or line.startswith('--- a/') or line.startswith('+++ b/'):
        out.append(line)
        continue
    if line.startswith('@@ '):
        out.append(line)
        continue
    
    if line.startswith('+') or line.startswith('-') or line.startswith(' '):
        out.append(line)
    else:
        # continuation line
        if out:
            out[-1] = out[-1] + line
        else:
            out.append(line)

with open('/tmp/unwrapped.diff', 'w') as f:
    f.write('\n'.join(out) + '\n')
