#!/usr/bin/env bash
# Export a clean, self-contained navigation source bundle on demand.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash tools/export_navigation_module.sh 2d /target/directory
  bash tools/export_navigation_module.sh 3d /target/directory [--with-baseline]

The target must be absent or empty. Exports contain only the active source and
publicly shareable documentation. Historical experiments, private real-robot
material, runtime traces, 2D material (for a 3D export), and article PDFs are
always excluded. `--with-baseline` adds a minimal Record 9 simulation-only
checkpoint, not the full experiment directory.
EOF
}

if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage >&2
  exit 2
fi

module="$1"
target="$2"
baseline="false"
if [[ $# -eq 3 ]]; then
  [[ "$3" == "--with-baseline" ]] || { usage >&2; exit 2; }
  baseline="true"
fi
[[ "$module" == "2d" || "$module" == "3d" ]] || { usage >&2; exit 2; }
[[ "$module" == "3d" || "$baseline" == "false" ]] || {
  echo "--with-baseline is only available for the 3d export." >&2
  exit 2
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_hint="${SCOUT_WS_ROOT:-${CATKIN_WS:-${WS_ROOT:-}}}"
if [[ -n "${workspace_hint}" ]]; then
  workspace_root="$(cd "${workspace_hint}" && pwd)"
else
  workspace_root="$(cd "${script_dir}/.." && pwd)"
fi

[[ -d "${workspace_root}/ros2_ws/src" ]] || {
  echo "Not a Scout workspace: ${workspace_root}" >&2
  exit 2
}
if [[ -e "$target" && -n "$(find "$target" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Refusing to overwrite a non-empty export target: $target" >&2
  exit 2
fi
mkdir -p "$target"
target="$(cd "$target" && pwd)"

excludes=(
  --exclude '__pycache__/'
  --exclude '.pytest_cache/'
  --exclude '*.pyc'
  --exclude '*.pyo'
  --exclude '.git/'
  --exclude 'build/'
  --exclude 'install/'
  --exclude 'log/'
)

public_excludes=(
  --exclude '*.log'
  --exclude '.last*'
  --exclude '.boundary_live.json'
  --exclude '*.bak*'
  --exclude '*.env'
  --exclude '*.pem'
  --exclude '*.key'
  --exclude 'real_robot_*.yaml'
)

copy_tree() {
  local source="$1"
  local destination="$2"
  shift 2
  [[ -d "$source" ]] || { echo "Missing required source: $source" >&2; exit 1; }
  mkdir -p "$destination"
  rsync -a "${excludes[@]}" "${public_excludes[@]}" "$@" "${source}/" "${destination}/"
}

copy_file() {
  local source="$1"
  local destination="$2"
  [[ -f "$source" ]] || { echo "Missing required file: $source" >&2; exit 1; }
  mkdir -p "$(dirname "$destination")"
  cp -p "$source" "$destination"
}

# A standalone export has no copy of the workspace archive.  That is not a
# reason to shorten, replace, or otherwise rewrite the detailed 3D manuals:
# their formulas, reasoning, command sequences, validation evidence, and
# plain-text historical references are project assets.  The export-local root
# README explains what is absent; the 3D manuals themselves are kept intact.
# Only a genuinely runtime-relevant registry path is adapted below.
rewrite_exported_docs() {
  local export_module="$1"
  local export_root="$2"
  local include_baseline="$3"

  if [[ "$export_module" == "2d" ]]; then
    sed -i \
      '/^旧的分册手册、文件清单和调试流水账已原样归档到 /c\历史分册手册和调试流水账未随独立导出提供；请仅使用本仓库当前 README 和脚本，勿将历史命令直接用于本模块。' \
      "${export_root}/2D/README.md"
    sed -i \
      '/^替换当前入口前的文档和调试记录保存在 /c\替换当前入口前的文档和调试记录未随独立导出提供；请仅使用本仓库当前文件作为运行依据。' \
      "${export_root}/2D/HISTORY.md"
    return
  fi

  # Deliberately no 3D Markdown rewrite here.  In particular, do not replace
  # sections after a matching heading and do not remove archive references:
  # they document provenance even when the archived material is excluded.
  # The model registry is handled separately because it is runtime input, not
  # explanatory documentation.
  : "${include_baseline}"
}

write_export_root_docs() {
  local export_module="$1"
  local export_root="$2"
  local include_baseline="$3"

  if [[ "$export_module" == "2d" ]]; then
    printf '%s\n' \
      "# Scout 2D navigation export" \
      "" \
      "This is a standalone ROS2 Humble source bundle for the 2D indoor navigation module." \
      "" \
      "## Scope" \
      "" \
      "- \`2D/\`: current documentation, scripts, and maps." \
      "- \`ros2_ws/src/navigation_2d/scout_offroad_2d/\`: the 2D ROS package." \
      "- \`ros2_ws/src/shared/\`: the one shared vehicle-model and path-tool copy required by this module." \
      "" \
      "Historical material, private deployment material, runtime logs, and the 3D navigation module are intentionally absent." \
      "Build from \`ros2_ws/\`; start with \`2D/README.md\` and follow \`AGENTS.md\`." \
      "" \
      "Before publishing, choose a project license and confirm that all included model assets may be redistributed." \
      > "${export_root}/README.md"
    printf '%s\n' \
      "# Standalone 2D export rules" \
      "" \
      "This export contains only the active 2D navigation package and its shared ROS dependencies." \
      "- Keep compilable code below \`ros2_ws/src/\`; build from \`ros2_ws/\`." \
      "- Resolve runtime paths through \`SCOUT_WS_ROOT\`, package share paths, or the supplied workspace layout; do not add user-home hard-coded paths." \
      "- Keep logs and generated runs out of source control. Do not add private deployment data, credentials, or the other navigation module." \
      "- Default verification is simulation-only; preserve the module boundary recorded in \`MODULE_MAP.md\`." \
      > "${export_root}/AGENTS.md"
    printf '%s\n' \
      "# 2D export module map" \
      "" \
      "\`2D/\` holds current 2D documentation, scripts, maps, and non-source resources." \
      "\`ros2_ws/src/navigation_2d/scout_offroad_2d/\` is the only 2D navigation package." \
      "\`ros2_ws/src/shared/\` holds the shared vehicle description and workspace path helper required by this export." \
      "" \
      "The 3D module, historical records, private deployment material, and build/runtime outputs are intentionally not part of this repository." \
      > "${export_root}/MODULE_MAP.md"
    return
  fi

  printf '%s\n' \
    "# Scout 3D navigation export" \
    "" \
    "This is a standalone ROS2 Humble + Gazebo Classic source bundle for the 3D TD3 and long-navigation chain." \
    "" \
    "## Scope" \
    "" \
    "- \`3D/\`: current documentation, scripts, maps, and UI resources." \
    "- \`ros2_ws/src/navigation_3d/\`: the TD3 and long-navigation ROS packages." \
    "- \`ros2_ws/src/shared/\`: the one shared vehicle-model and path-tool copy required by this module." \
    "- \`ros2_ws/src/third_party/\`: dependency notes and a guarded optional dependency clone script." \
    "" \
    "Historical material, private deployment material, runtime traces, and the 2D navigation module are intentionally absent." \
    "The complete detailed manuals in \`3D/README*.md\` are retained without simplification. Their plain-text archive references are provenance notes; the referenced history is not bundled in this export." \
    "Build from \`ros2_ws/\`; start with \`3D/README.md\`, \`3D/DELIVERY_MANIFEST.md\`, and \`AGENTS.md\`." \
    "" \
    "Before publishing, choose a project license and confirm that all included model assets and third-party notices may be redistributed." \
    > "${export_root}/README.md"
  if [[ "$include_baseline" == "true" ]]; then
    printf '%s\n' \
      "" \
      "A minimal Record 9 simulation-only baseline is included under \`3D/baselines/record9_simulation/\`." \
      >> "${export_root}/README.md"
  else
    printf '%s\n' \
      "" \
      "This default export does not bundle Record 9 weights. Train a new run or re-export with \`--with-baseline\` before using inference examples that require a checkpoint." \
      >> "${export_root}/README.md"
  fi
  printf '%s\n' \
    "# Standalone 3D export rules" \
    "" \
    "This export contains only the active 3D TD3 and long-navigation packages and their shared ROS dependencies." \
    "- Keep compilable code below \`ros2_ws/src/\`; build from \`ros2_ws/\`." \
    "- Resolve runtime paths through \`SCOUT_WS_ROOT\`, package share paths, or the supplied workspace layout; do not add user-home hard-coded paths." \
    "- Keep logs, generated runs, private deployment data, and credentials out of source control." \
    "- Training starts fresh by default; resume requires an explicit compatible checkpoint. All real-robot control remains off by default." \
    "- Preserve complete formulas, reasoning, operation steps, and validation evidence in the detailed manuals. Do not replace them with a short index or remove historical provenance merely because this export omits the archive." \
    "- Preserve the module boundary recorded in \`MODULE_MAP.md\`; do not add the 2D module or historical material to this repository." \
    > "${export_root}/AGENTS.md"
  printf '%s\n' \
    "# 3D export module map" \
    "" \
    "\`3D/\` holds current 3D documentation, scripts, maps, and UI resources." \
    "\`ros2_ws/src/navigation_3d/scout_offroad_3d/\` is the TD3 local-navigation package." \
    "\`ros2_ws/src/navigation_3d/scout_long_nav/\` is the long-navigation package." \
    "\`ros2_ws/src/shared/\` holds the shared vehicle description and workspace path helper required by this export." \
    "\`ros2_ws/src/third_party/\` contains optional dependency notes only, not vendored external source." \
    "" \
    "The 2D module, historical records, private deployment material, and build/runtime outputs are intentionally not part of this repository." \
      > "${export_root}/MODULE_MAP.md"
}

write_export_ros2_rules() {
  local export_module="$1"
  local export_root="$2"

  if [[ "$export_module" == "2d" ]]; then
    printf '%s\n' \
      "# ROS2 workspace rules for the standalone 2D export" \
      "" \
      "The export-root \`AGENTS.md\` and \`MODULE_MAP.md\` define this repository's boundary." \
      "- Add compilable code only under \`src/shared/\` or \`src/navigation_2d/\`." \
      "- Keep the shared vehicle description and path helper single-sourced under \`src/shared/\`." \
      "- Resolve paths through \`SCOUT_WS_ROOT\` or ROS package-share locations; do not add user-home hard-coded paths." \
      "- Build from this directory. Keep \`build/\`, \`install/\`, \`log/\`, credentials, and generated diagnostics out of source control." \
      "- Verification is simulation-only unless a separately reviewed deployment workflow explicitly says otherwise." \
      > "${export_root}/ros2_ws/AGENTS.md"
    return
  fi

  printf '%s\n' \
    "# ROS2 workspace rules for the standalone 3D export" \
    "" \
    "The export-root \`AGENTS.md\` and \`MODULE_MAP.md\` define this repository's boundary." \
    "- Add compilable code only under \`src/shared/\`, \`src/navigation_3d/\`, or the documented \`src/third_party/\` dependency area." \
    "- Keep the shared vehicle description and path helper single-sourced under \`src/shared/\`." \
    "- Resolve paths through \`SCOUT_WS_ROOT\` or ROS package-share locations; do not add user-home hard-coded paths." \
    "- Build from this directory. Keep \`build/\`, \`install/\`, \`log/\`, credentials, generated runs, and diagnostics out of source control." \
    "- Training starts fresh by default; real-robot control stays off by default. Basic verification is simulation-only." \
    > "${export_root}/ros2_ws/AGENTS.md"
}

assert_exported_docs_are_standalone() {
  local export_root="$1"
  [[ -f "${export_root}/AGENTS.md" && -f "${export_root}/MODULE_MAP.md" ]] || {
    echo "Standalone export governance documents are missing." >&2
    exit 1
  }
  if [[ -d "${export_root}/3D" ]]; then
    for detailed_manual in README.md README0.md README1.md README2.md README3.md README4.md; do
      [[ -f "${export_root}/3D/${detailed_manual}" ]] || {
        echo "Detailed 3D manual is missing from export: ${detailed_manual}" >&2
        exit 1
      }
    done
  fi
}

copy_tree "${workspace_root}/ros2_ws/src/shared/scout_description" \
  "${target}/ros2_ws/src/shared/scout_description"
copy_tree "${workspace_root}/ros2_ws/src/shared/scout_mini_pro_description" \
  "${target}/ros2_ws/src/shared/scout_mini_pro_description"
copy_tree "${workspace_root}/ros2_ws/src/shared/scout_workspace_common" \
  "${target}/ros2_ws/src/shared/scout_workspace_common"

if [[ "$module" == "2d" ]]; then
  # Runtime diagnostics belong to the source workspace, not a portable module.
  copy_tree "${workspace_root}/2D" "${target}/2D" --exclude 'debug_logs/'
  copy_tree "${workspace_root}/ros2_ws/src/navigation_2d/scout_offroad_2d" \
    "${target}/ros2_ws/src/navigation_2d/scout_offroad_2d"
else
  mkdir -p "${target}/3D"
  rsync -a "${excludes[@]}" "${public_excludes[@]}" \
    --exclude 'runs/' \
    --exclude 'runs_long/' \
    --exclude 'debug_logs/' \
    --exclude 'gzsim_demo/' \
    --exclude 'REAL_ROBOT_HANDOFF.md' \
    --exclude 'scripts/jammy_post_upgrade_setup.sh' \
    "${workspace_root}/3D/" "${target}/3D/"
  copy_tree "${workspace_root}/ros2_ws/src/navigation_3d/scout_offroad_3d" \
    "${target}/ros2_ws/src/navigation_3d/scout_offroad_3d"
  copy_tree "${workspace_root}/ros2_ws/src/navigation_3d/scout_long_nav" \
    "${target}/ros2_ws/src/navigation_3d/scout_long_nav"
  for third_party_file in README.md BUILD_NOTES.md clone_fast_lio.sh; do
    copy_file "${workspace_root}/ros2_ws/src/third_party/${third_party_file}" \
      "${target}/ros2_ws/src/third_party/${third_party_file}"
  done

  # The active registry intentionally names the workspace's historical Record
  # 9 location.  A public export must not retain that unavailable reference:
  # both export variants use a portable optional location below.  Only the
  # --with-baseline variant actually copies the checkpoint there.
  exported_registry="${target}/3D/scout_nav_app/config/models.yaml"
  exported_app_readme="${target}/3D/scout_nav_app/README.md"
  source_registry_path='../../../archive/experiments/td3/runs/run_20260907_120841'
  export_registry_path='../../baselines/record9_simulation'
  [[ -f "${exported_registry}" && -f "${exported_app_readme}" ]] || {
    echo "Missing exported ScoutNav model documentation or registry." >&2
    exit 1
  }
  grep -Fq "path: \"${source_registry_path}\"" "${exported_registry}" || {
    echo "Expected Record 9 source entry is absent from ${exported_registry}" >&2
    exit 1
  }
  sed -i "s|path: \"${source_registry_path}\"|path: \"${export_registry_path}\"|" \
    "${exported_registry}"
  sed -i '1i# Public export: Record 9 is optional at the portable path below.\n# It is bundled only with --with-baseline; otherwise select a compatible external model.' \
    "${exported_registry}"

  # Keep the copied application README coherent with the copied registry.  Do
  # not alter the workspace README, which correctly records the local source
  # layout.  Preserve its historical prose; only adapt the concrete baseline
  # location that is presented as an exported runtime path.
  sed -i \
    -e 's|`archive/experiments/td3/runs/run_20260907_120841/`|`3D/baselines/record9_simulation/`|' \
    "${exported_app_readme}"
  if [[ "$baseline" == "true" ]]; then
    printf '%s\n' \
      "" \
      "## Exported baseline availability" \
      "" \
      "This export includes Record 9 at \`3D/baselines/record9_simulation/\`. \`config/models.yaml\` uses this portable relative location. It remains simulation-only." \
      >> "${exported_app_readme}"
  else
    printf '%s\n' \
      "" \
      "## Exported baseline availability" \
      "" \
      "This default public export intentionally excludes model weights. Re-export with \`--with-baseline\` to provide Record 9 at \`3D/baselines/record9_simulation/\`, or select a compatible external checkpoint. The model loader safely rejects a missing directory." \
      >> "${exported_app_readme}"
  fi
  if grep -Fq "${source_registry_path}" "${exported_registry}"; then
    echo "Exported ScoutNav registry still contains the local Record 9 runtime path." >&2
    exit 1
  fi

  if [[ "$baseline" == "true" ]]; then
    baseline_source="${workspace_root}/archive/experiments/td3/runs/run_20260907_120841"
    baseline_target="${target}/3D/baselines/record9_simulation"
    for baseline_file in td3_actor td3_actor_optimizer td3_critic td3_critic_optimizer config.yaml params_backup.yaml; do
      copy_file "${baseline_source}/${baseline_file}" "${baseline_target}/${baseline_file}"
    done
    printf '%s\n' \
      "# Record 9 simulation baseline" \
      "" \
      "This is a minimal compatibility bundle: 150-dimensional state, hidden_dim 1024." \
      "It is an experimental simulator checkpoint only and must not control a real robot." \
      "The complete historical experiment remains outside this export." \
      > "${baseline_target}/BASELINE.md"
  fi
fi

copy_file "${workspace_root}/.gitignore" "${target}/.gitignore"
rewrite_exported_docs "${module}" "${target}" "${baseline}"
write_export_root_docs "${module}" "${target}" "${baseline}"
write_export_ros2_rules "${module}" "${target}"
assert_exported_docs_are_standalone "${target}"

echo "Export ready: ${target}"
