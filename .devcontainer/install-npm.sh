#!/usr/bin/env bash

# Combine all NVM commands into one RUN statement
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash && \
    export NVM_DIR="$HOME/.nvm" && \
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" && \
    nvm install 22 && \
    nvm use 22

# Ensure nvm is sourced for subsequent commands by using the shell option
\. "$HOME/.nvm/nvm.sh"

# Install global npm packages
npm install -g @anthropic-ai/claude-code