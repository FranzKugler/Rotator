# syntax=docker/dockerfile:1.7
FROM node@sha256:8a34c4ab3ea2c5cd194f07e317b2a8f09461d3c8b05c4e34c8ccd56d56024c4d AS node-runtime
FROM espressif/idf@sha256:6b4adab7b282e9261a154d7130fb4945a3c28bd35c2e914357c2f522643cb92a

ARG DEV_UID=1000
ARG DEV_GID=1000
ARG CLAUDE_CODE_VERSION=2.1.247

ENV DEBIAN_FRONTEND=noninteractive \
    HOME=/home/developer \
    npm_config_cache=/home/developer/.npm \
    IDF_CCACHE_ENABLE=1 \
    CCACHE_DIR=/home/developer/.cache/ccache \
    IDF_PYTHON_ENV_PATH=/opt/esp/python_env/idf5.4_py3.12_env \
    ESP_ROM_ELF_DIR=/opt/esp/tools/esp-rom-elfs/20241011/ \
    OPENOCD_SCRIPTS=/opt/esp/tools/openocd-esp32/v0.12.0-esp32-20241016/openocd-esp32/share/openocd/scripts \
    PATH=/home/developer/node_modules/.bin:/opt/esp/idf/components/espcoredump:/opt/esp/idf/components/partition_table:/opt/esp/idf/components/app_update:/opt/esp/tools/xtensa-esp-elf-gdb/14.2_20240403/xtensa-esp-elf-gdb/bin:/opt/esp/tools/riscv32-esp-elf-gdb/14.2_20240403/riscv32-esp-elf-gdb/bin:/opt/esp/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin:/opt/esp/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin:/opt/esp/tools/esp32ulp-elf/2.38_20240113/esp32ulp-elf/bin:/opt/esp/tools/cmake/3.30.2/bin:/opt/esp/tools/openocd-esp32/v0.12.0-esp32-20241016/openocd-esp32/bin:/opt/esp/tools/qemu-xtensa/esp_develop_9.0.0_20240606/qemu/bin:/opt/esp/tools/qemu-riscv32/esp_develop_9.0.0_20240606/qemu/bin:/opt/esp/python_env/idf5.4_py3.12_env/bin:/opt/esp/idf/tools:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

USER root

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       bash-completion ca-certificates curl git jq openssh-client tini udev \
    && rm -rf /var/lib/apt/lists/* \
    && if getent group "${DEV_GID}" >/dev/null; then \
         group_name="$(getent group "${DEV_GID}" | cut -d: -f1)"; \
       else \
         groupadd --gid "${DEV_GID}" developer; group_name=developer; \
       fi \
    && if getent passwd "${DEV_UID}" >/dev/null; then \
         old_user="$(getent passwd "${DEV_UID}" | cut -d: -f1)"; \
         usermod --login developer --home /home/developer --move-home --shell /bin/bash "$old_user"; \
         usermod --gid "$group_name" --groups "$group_name" developer; \
       else \
         useradd --uid "${DEV_UID}" --gid "$group_name" --create-home --shell /bin/bash developer; \
       fi \
    && mkdir -p /home/developer/.npm /home/developer/.claude /home/developer/.cache/ccache /home/developer/.ssh /workspace \
    && chown -R developer:"$group_name" /home/developer /workspace

# Claude Code requires Node 22. Copy the pinned official Node runtime instead
# of using Ubuntu's older distribution package.
COPY --from=node-runtime /usr/local/ /usr/local/

RUN npm install -g --allow-scripts=@anthropic-ai/claude-code "@anthropic-ai/claude-code@${CLAUDE_CODE_VERSION}"

COPY --chown=developer:ubuntu package.json package-lock.json /tmp/rotator-npm/
RUN mkdir -p /home/developer/node_modules \
    && cd /tmp/rotator-npm \
    && npm ci \
    && cp -a node_modules/. /home/developer/node_modules/ \
    && chown -R developer:ubuntu /home/developer/node_modules \
    && rm -rf /tmp/rotator-npm

USER developer
WORKDIR /workspace

RUN git config --global --add safe.directory /workspace \
    && git config --global --add safe.directory /opt/esp/idf/components/openthread/openthread

ENTRYPOINT ["/usr/bin/tini", "--", "/opt/esp/entrypoint.sh"]
CMD ["sleep", "infinity"]
