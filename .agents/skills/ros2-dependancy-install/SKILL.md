---
name: ros2-dependancy-install
description: Manages adding ROS 2 dependencies (libraries, drivers, tools, other ROS packages) to a colcon workspace that runs inside a devcontainer/Docker setup. Use this skill whenever the user wants to add, install, vendor, or wire up a new dependency for a ROS 2 project with a `src/` colcon workspace and a `.devcontainer/Dockerfile` — even if phrased casually, e.g. "I need cv_bridge in this project", "add this driver to my robot stack", "how do I get OpenCV into the container", "this node needs libfoo-dev", "vendor this repo into my workspace", or "pin this package to a fork". Always consult this skill before manually hand-editing package.xml, the Dockerfile, or a .repos file for a ROS 2 project — it encodes the correct order of operations (package.xml/rosdep first, source-level vendoring second, a direct system-level Dockerfile install — apt/pip or build-from-source — only when neither applies) and will keep the final container image slim while still usable by a developer.
---

# ROS 2 Devcontainer Dependency Manager

This skill walks through *how* a new dependency should enter a ROS 2 devcontainer
project, in priority order. Lower-priority paths are only taken when the one above
it genuinely doesn't apply — don't skip ahead because it's easier.

> [!TIP]
> You MUST NEVER run `apt install` or `pip install` directly in the Docker container or the terminal. This will cause the container to diverge from the Dockerfile and break reproducibility. Instead, follow the steps below to add a dependency in a way that will be reproducible for all developers.

> [!CAUTION]
> Avoid adding dependencies to the Dockerfile that could be resolved by rosdep. Making modifications to the Dockerfile instead of other routes such as the package.xml or .repos file will trigger a long CI build time and will result in a larger final image size. If you have to modify the Dockerfile, the developer session must be restarted. Treat the Dockerfile as if it is hot lava. Only use the Dockerfile route when rosdep cannot resolve the dependency.


```
New dependency needed
        │
        ▼
1. Find target package(s)
        │
        ▼
2. Search for existing rosdep key
        │
        ▼
3. Can rosdep resolve it?
        │
   yes ─┴─► add package.xml dependency
        │
        no
        ▼
4. Is there an existing ROS vendor package?
        │
   yes ─┴─► use vendor package
        │
        no
        ▼
5. Is this a source repository?
        │
   yes ─┴─► .repos (preferred)
        │
        │
        └─► submodule only when:
             - repository is private
             - vcs import cannot access it
             - repository must be versioned with this project
             - user explicitly requests submodules
        │
        no
        ▼
6. Dockerfile install
        │
        ▼
7. Verify
```

## Step 1 — Find or choose the target package(s)

1. Locate the workspace root and list packages:
   ```bash
   find src -maxdepth 3 -name package.xml
   ```
2. **Zero packages found:** there's no package to own this dependency yet. Propose
   creating one with `ros2 pkg create`. Infer the build type from the dependency/task:
   - Pure Python node/library → `--build-type ament_python`
   - C++ or mixed → `--build-type ament_cmake`
   If it's genuinely unclear, ask the user rather than guessing the package name/type.
3. **One package found:** use it.
4. **Multiple packages found:** decide which package(s) this belongs to using this
   priority order, stopping at the first one that gives a clear answer:
   1. **Where the relevant code is actually being written right now** — if you're
      actively adding or editing code that will use this dependency, the package
      containing that code is the answer.
   2. **What the developer is doing**, if you aren't the one writing the code —
      look at the active task, the files they have open/are editing, the
      node/launch file they're running, etc.
   3. **Each package's `<description>`** in `package.xml` — use this only as a
      tie-breaker once the above two don't settle it.
   A dependency needed by more than one package gets declared in *each* of their
   `package.xml` files — don't dedupe it into just one.

## Step 2 — Discover dependencies

Before assuming a dependency name, search for existing ROS packages and rosdep keys.

```
rosdep search <term>
```

Examples:

```
rosdep search opencv
rosdep search yaml
rosdep search pcl
```

For ROS packages:

```
apt-cache search ros-${ROS_DISTRO}
```

or

```
ros2 pkg list | grep <name>
```

Only proceed once a likely package or rosdep key has been identified.

## Step 3 — Try rosdep first (the preferred, default outcome)

This is where most dependencies should end up. Only move past this step if rosdep
genuinely cannot resolve the key.

1. Make sure the rosdep database isn't stale: `rosdep update` (safe to run anytime).
2. Check resolvability:
   ```bash
   rosdep resolve <key>
   rosdep what-needs <key>
   ```
   If this prints a resolved package name, it's manageable. If it errors with
   "Cannot locate rosdep definition for...", it isn't — proceed to Step 3.
   - The rosdep key isn't always identical to the apt/pip name. If a plausible key
     fails, try variants (e.g. `cv_bridge`, `python3-opencv`) before concluding it's
     unresolvable.
3. Add the dependency to the chosen package(s)' `package.xml`. For rosdep-resolved
   dependencies, the real choice is almost always between two tags:
   - `<depend>` — needed at both build *and* run time (default for most libraries)
   - `<exec_depend>` — needed only at runtime (e.g. invoked by a launch file, a CLI
     tool, a runtime-only Python dependency)
   The other tags below exist for narrower cases — reach for them only when one
   specifically applies, not by default.
4. Sanity-check that the Dockerfile actually installs rosdep deps somewhere before
   the `final` stage (typically `rosdep install --from-paths src --ignore-src -r -y`).
   If that line is missing entirely, flag it to the user — that's a separate,
   pre-existing gap, don't silently restructure the Dockerfile to add it without
   confirming.
5. **Stop here.** No Dockerfile or submodule changes needed.

### Reference: which package.xml tag to use

| Tag | Use when the dependency is needed... |
|---|---|
| `<depend>` ⭐ primary | At both build *and* run time (most common case for libraries) |
| `<exec_depend>` ⭐ primary | Only at runtime (e.g. a launch-time Python dependency, a CLI tool invoked by a node) |
| `<build_depend>` | Only to build this package (e.g. a codegen tool) |
| `<build_export_depend>` | Needed by *downstream* packages that build against this one (e.g. exposed headers) |
| `<test_depend>` | Only for this package's tests |

## Step 3 — Source-level dependency? Decide submodule vs. `.repos`

This step is for dependencies that are themselves a ROS package / git repo with
source code that needs to be built — not a plain system library. Skip straight to
Step 4 if it's a system library/tool with no relevant source repo.

**Decide: will a developer plausibly need to read, modify, patch, or debug this
source from inside this workspace?** Signals it should be a submodule:
- The user is explicitly planning to patch/fork/contribute to it
- It's unreleased/bleeding-edge with no usable binary release
- Day-to-day development on *this* project involves stepping into its code

If none of those apply — it's consumed as-is, like any other build dependency —
prefer the `.repos` route. When genuinely ambiguous, ask the user; don't guess on
something that determines whether their diffs show up in `git status`.

### 3a. Submodule path (dev needs to touch the source)

```bash
git submodule add <repo-url> src/<name>
```
Once it's in `src/`, it's just another package in the workspace — loop back to
**Step 1/2** for its own `package.xml` dependencies too (resolve those the same way,
recursively).

### 3b. `.repos` + vcs import path (consumed as-is)

1. Check whether a `.repos` file already exists somewhere in the repo (root,
   `.devcontainer/`, etc.) and matches an existing naming convention — reuse it if so.
2. Otherwise create one with a name that reflects its contents (e.g. `deps.repos`,
   `vendor.repos`) rather than always defaulting to a generic name.
3. Add an entry, e.g.:
   ```yaml
   repositories:
     <name>:
       type: git
       url: <repo-url>
       version: <branch-or-tag>
   ```
4. Wire it into the Dockerfile **before the `final` stage**, matching whatever
   `WORKDIR` the Dockerfile actually uses (don't assume `/workspace` — check):
   ```dockerfile
   COPY <file>.repos <workdir>/
   RUN mkdir -p <workdir>/src \
       && vcs import <workdir>/src < <workdir>/<file>.repos
   ```

## Step 4 — Check for Existing Vendor Packages

Before adding external repositories, determine whether a ROS vendor package already exists.

Examples:

```
tinyxml2_vendor
asio_vendor
spdlog_vendor
cyclonedds_vendor
fastcdr
```

If a maintained vendor package exists:

```
<depend>tinyxml2_vendor</depend>
```

is preferred over importing upstream source.

Reasons:

- Already maintained by ROS ecosystem
- Distribution-compatible
- Security updates handled upstream
- Less maintenance burden

Only continue if no suitable vendor package exists.

## Step 5 — System-level Dockerfile install: apt/pip, or build from source

This path is for dependencies that are **not** a ROS package/source repo at all
(Step 3 doesn't apply) and that rosdep **genuinely cannot resolve** (checked in
Step 2, not assumed). It's a distinct, legitimate path on its own terms — not a
degraded fallback — but it always requires a documented reason, since it bypasses
the package.xml/rosdep mechanism most dependencies should use.

### 5a.Before Editing Dockerfile

Confirm:

```
rosdep search <dependency>
```

and

```
rosdep resolve <dependency>
```

have already failed. Document the reason directly above the install step.
```
# No rosdep key exists for vendor SDK
RUN apt-get update && apt-get install -y vendor-sdk
```

1. **Write the reason down explicitly** and show it to the user before editing
   anything. Valid reasons look like: "only ships as a vendor-provided .deb not in
   any rosdep-known repo", "requires a license-gated installer", "no rosdep entry
   and no packaged release — must be built from source". Not valid: "didn't check
   rosdep" / convenience — go back to Step 2 if so.
2. Find `.devcontainer/Dockerfile` and confirm it has a multi-stage build with a
   `final` target (`grep -i "as final"`).
   - **If there's no `final` stage at all: stop and ask the user how they want to
     proceed before restructuring the Dockerfile.** Don't introduce a multi-stage
     build on your own initiative.
3. Then pick whichever of the two sub-paths below actually matches — they are
   separate situations, not interchangeable defaults.

### 5b. Package-manager install (apt-get / pip)

Use this when an installable artifact already exists — an apt package, a `.deb`,
a PyPI package — and the only problem is that rosdep doesn't have an entry for it.

```dockerfile
# rosdep cannot resolve this: <reason>
RUN apt-get update && apt-get install -y <package>
```

### 5c. Build from source / install a tarball

Use this when there's no installable package at all — only source code to clone
and build, or a tarball to download and extract — and it's being installed as a
system-level artifact, *not* as a colcon-built workspace package (if it should be
buildable by colcon instead, that's Step 3, not this).

```dockerfile
# rosdep cannot resolve this: <reason>
RUN git clone <repo-url> /opt/<name> \
    && cmake -S /opt/<name> -B /opt/<name>/build \
    && cmake --build /opt/<name>/build --target install
# or, for a tarball:
# ADD <tarball-url> /opt/
# RUN tar -xf /opt/<tarball-name> -C /opt/ && <build/install steps>
```

Whichever sub-path applies, the install step must land in a stage *before* `final`
(or otherwise ensure the layer is actually present in the `final` image — the goal
is a slim final image that nonetheless contains everything a developer will need
to use the package), with the written reason kept as a comment directly above the
install line, as shown above.

New Verification Stage

This is the largest missing piece.

## Step 7 - Never Duplicate Dependency Sources

If a dependency is provided through:
```
<depend>foo</depend>
```

do not also:

```
RUN apt install foo
```

and do not also:
```
repositories:
  foo:
```

Choose **exactly one source of truth**.

Priority:

```
package.xml + rosdep
    >
vendor package
    >
.repos
    >
submodule
    >
Dockerfile
```

A dependency should enter the project through the highest applicable layer and only that layer.

## Step 8 — Verify the Dependency

After any dependency change:

### 8a. Validate rosdep

```
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

### 8b. Check unresolved dependencies
```
rosdep check \
  --from-paths src \
  --ignore-src
```

### 8c. Build workspace
```
colcon build
```

### 8d. Run tests
```
colcon test
```

### 8e. If Dockerfile was modified:

```
docker build .
```

It is also wise to inform the user that they will need to restart their devcontainer session after a Dockerfile change.

## After making changes: report back

Always close out with a short summary: which package(s)' `package.xml` were edited
and with which tag, whether rosdep resolved the dependency or a system-level
Dockerfile install was used instead (via apt/pip or build-from-source, and why),
and whether a submodule, `.repos` file, or the Dockerfile itself was touched.
