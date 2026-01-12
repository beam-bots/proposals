# Proposal: Usage Rules for Beam Bots Packages

**Status:** Draft
**Author:** James Harton
**Created:** 2026-01-13

---

## Summary

Add `usage-rules.md` files to published Beam Bots packages to provide AI coding assistants with authoritative, concise guidance for users building robots with BB. This leverages the `usage_rules` hex package to make LLM-friendly documentation discoverable via `mix usage_rules.docs`.

---

## Motivation

AI coding assistants frequently help users integrate with libraries, but without authoritative guidance they may:

- Generate incorrect or non-idiomatic code
- Miss critical safety requirements (e.g., arming before commands)
- Use deprecated patterns or bypass framework conventions
- Confuse internal APIs with public interfaces

The Ash Framework pioneered `usage-rules.md` files that ship with hex packages. Users can run `mix usage_rules.docs ash` to get maintainer-curated guidance. This proposal brings the same pattern to Beam Bots.

**Key distinction:**
- `AGENTS.md` = guidance for AI assistants working *on* the codebase (development)
- `usage-rules.md` = guidance for AI assistants helping users *use* the library (consumption)

---

## Design

### Packages Requiring Usage Rules

**Core packages:**
- `bb` - Core framework DSL, supervision, runtime, kinematics
- `bb_liveview` - Phoenix LiveView dashboard
- `bb_kino` - Livebook widgets
- `bb_reactor` - Reactor integration

**IK solvers:**
- `bb_ik_dls` - Damped Least Squares solver
- `bb_ik_fabrik` - FABRIK solver

**Servo drivers:**
- `bb_servo_robotis` - Dynamixel servos
- `bb_servo_pca9685` - PCA9685 PWM driver
- `bb_servo_pigpio` - Raspberry Pi GPIO PWM
- `bb_servo_feetech` - Feetech servos

### File Structure

Each `usage-rules.md` follows this template (~50-150 lines):

```markdown
# {Package} Usage Rules

Brief one-line description.

## Core Principles
3-5 numbered rules

## Quick Start
Minimal working example (10-20 lines)

## Common Patterns
2-4 patterns with brief code examples

## Anti-patterns
2-3 things to avoid

## Quick Reference
Table of common operations

## Further Reading
Links to hexdocs, tutorials
```

### Design Principles

1. **Succinct over comprehensive** - Respect token budgets; ~100-200 lines max per package
2. **Reference, don't duplicate** - Point to hexdocs and tutorials for detail
3. **Patterns and anti-patterns** - Focus on what AI assistants get wrong
4. **Quick reference tables** - Enable fast lookup of common operations

### Example: bb/usage-rules.md

```markdown
# Beam Bots (BB) Usage Rules

Framework for building resilient robotics projects in Elixir using Spark DSL.

## Core Principles

1. **Define robots with the DSL** - Use `use BB` with topology, not manual GenServers
2. **Physical units use ~u sigil** - Always `~u(90 degree)` not `1.57`
3. **Use arm/disarm commands** - `BB.Robot.Runtime.arm/1` runs prearm checks; don't call Safety directly
4. **Supervision mirrors topology** - Crashes isolate to affected subtree
5. **Use BB.Message for payloads** - Wraps data with timestamps and frame IDs

## Quick Start

defmodule MyRobot do
  use BB

  topology do
    link :base do
      joint :pan, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree)
        actuator :servo, {MyDriver, servo_id: 1}
      end
    end
  end
end

# Start and arm
{:ok, _} = MyRobot.start_link(simulation: :kinematic)
{:ok, _} = BB.Robot.Runtime.arm(MyRobot)

# Command
BB.Actuator.set_position(MyRobot, :servo, 0.5)

## Common Patterns

### Subscribing to sensor data

BB.subscribe(MyRobot, [:sensor])

def handle_info({:bb, _path, %BB.Message{payload: payload}}, state) do
  # Handle JointState, BeginMotion, etc.
end

### Forward kinematics

positions = BB.Robot.Runtime.joint_positions(MyRobot)
{:ok, transforms} = BB.Robot.Kinematics.forward(MyRobot.robot(), positions)
tool_pose = transforms[:end_effector]

## Anti-patterns

### Don't: Command without arming

# Fails - robot starts disarmed
BB.Actuator.set_position(MyRobot, :servo, 0.5)

### Don't: Manipulate safety system directly

# Bad - bypasses prearm checks defined in Arm command
BB.Robot.Runtime.Safety.arm(MyRobot)

# Good - uses command system, runs user's prearm checks
BB.Robot.Runtime.arm(MyRobot)

### Don't: Use raw numbers for units

# Bad - ambiguous
limit lower: -1.57, upper: 1.57

# Good - explicit
limit lower: ~u(-90 degree), upper: ~u(90 degree)

### Don't: Bypass the DSL

# Bad - manual supervision
Supervisor.start_link([MyServo], strategy: :one_for_one)

# Good - let BB generate supervision tree
MyRobot.start_link()

## Quick Reference

| Task | Code |
|------|------|
| Start robot | `MyRobot.start_link()` |
| Simulation mode | `MyRobot.start_link(simulation: :kinematic)` |
| Arm | `BB.Robot.Runtime.arm(MyRobot)` |
| Disarm | `BB.Robot.Runtime.disarm(MyRobot)` |
| Set position | `BB.Actuator.set_position(robot, :name, radians)` |
| Get positions | `BB.Robot.Runtime.joint_positions(MyRobot)` |
| Subscribe | `BB.subscribe(MyRobot, [:sensor])` |
| Forward kinematics | `BB.Robot.Kinematics.forward(robot, positions)` |

## Further Reading

- [Tutorials](https://hexdocs.pm/bb/tutorials.html) - Step-by-step guides
- [DSL Reference](https://hexdocs.pm/bb/dsl-bb.html) - Complete DSL documentation
- [Safety System](https://hexdocs.pm/bb/safety.html) - Critical safety documentation
```

### Implementation Per Package

For each package:

1. Add `{:usage_rules, "~> 0.5", only: [:dev]}` to `mix.exs` deps
2. Create `usage-rules.md` in package root
3. Add `usage-rules.md` to `package/0` files list in `mix.exs`
4. Test with `mix usage_rules.docs {package}` locally
5. Publish new version to hex.pm

### Workspace Integration

Update the workspace `CLAUDE.md` to reference usage rules:

```markdown
## Documentation Lookup

Before implementing with BB packages, check usage rules:

mix usage_rules.docs bb                 # Core framework
mix usage_rules.docs bb_servo_robotis   # Servo driver
mix usage_rules.search_docs "inverse kinematics"

Local deps: `deps/<package>/usage-rules.md`
```

---

## Acceptance Criteria

### Must Have

- [ ] `bb` has `usage-rules.md` covering DSL, safety, commands, kinematics
- [ ] All servo driver packages have `usage-rules.md`
- [ ] All IK solver packages have `usage-rules.md`
- [ ] `bb_liveview` has `usage-rules.md`
- [ ] All files added to `package/0` files list for hex distribution
- [ ] Workspace `CLAUDE.md` updated to reference usage rules

### Should Have

- [ ] `bb_kino` has `usage-rules.md`
- [ ] `bb_reactor` has `usage-rules.md`

### Won't Have

- [ ] `bb_example_*` packages (apps, not libraries)
- [ ] Exhaustive documentation (that's what hexdocs is for)

---

## Open Questions

1. Should `usage_rules` be a runtime dependency or dev-only? Currently proposing dev-only since it's primarily for the `mix usage_rules.docs` task.

2. Should we add a CI check to ensure `usage-rules.md` exists and is included in package files?

---

## References

- [usage_rules package](https://hex.pm/packages/usage_rules) - The hex package enabling this
- [Ash usage-rules.md](https://github.com/ash-project/ash/blob/main/usage-rules.md) - Prior art from Ash Framework
- [synchronal/usage_rules](https://github.com/synchronal/usage_rules) - Source repository
