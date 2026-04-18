# Beyblade-style physics engine — learning roadmap

This document is a step-by-step path from general software development to a **spinning top in a curved stadium** (Beyblade-style), using your existing **GLFW + GLAD + OpenGL** bootstrap as the host process.

**Current starting point:** a window, a render loop (`processInput` → clear → `SwapBuffers` / `PollEvents`). Physics will be **state you update each step** and then **draw** (or debug-draw).

**Core habits:** small modules, vertical slices (each phase produces something runnable), and **debug visualization** (axes, normals, velocities) before chasing numeric perfection.

---

## Phase 0 — Mindset (about one afternoon)

1. **Separate three concerns** mentally and in code boundaries:
   - **Simulation** — numbers over time (positions, velocities, forces).
   - **Collision** — geometry queries and contact data (normals, points).
   - **Rendering** — drawing; should consume simulation state, not own physics rules.
2. Expect the first versions to **look wrong** until you can **draw debug lines**. Treat visualization as required infrastructure, not polish.

---

## Phase 1 — Comfort with the game loop (1–3 days)

1. Keep your current GLFW + GLAD window.
2. Add a **timer**: measure per-frame `deltaTime` (e.g. `glfwGetTime()` or `std::chrono`). You will refine how you use time in Phase 3; for now, get used to measuring it.
3. Add a minimal **“game object”** exercise: some CPU state that updates then influences what you draw or print—reinforcing **update → draw**.

**Goal:** You are comfortable with **update then draw** as the program rhythm.

---

## Phase 2 — 3D math (3–7 days)

Do this in a small `math` module (and on paper) before fighting the GPU.

1. **Vectors:** add, scale, dot, cross, length, normalize.
2. **Matrices:** translation / rotation / scale; matrix × vector; combining transforms.
3. **Quaternions (minimum):** identity, multiply, normalize, rotate a vector by a quaternion (slerp can wait).

**Goal:** You can transform points from **body/local space** to **world space** reliably.

---

## Phase 3 — Fixed timestep (about 1 day)

1. Understand why **fixed `dt`** helps stability and determinism compared to tying physics to variable frame time.
2. Implement an **accumulator pattern:** accumulate real elapsed time; while `accumulator >= fixed_dt`, run **one** physics step with `fixed_dt`, then subtract; cap **max steps per frame** to avoid spiral-of-death when the app stalls.

**Goal:** Similar motion at different frame rates (within reasonable tolerance).

---

## Phase 4 — First “engine” slices: 1D then 2D (3–5 days)

Start in lower dimensions on purpose.

1. **1D:** e.g. constant acceleration or a spring—track `x` and `v`, integrate with fixed step; log values and sanity-check by hand.
2. **2D:** circle vs axis-aligned box on the CPU—position, velocity, simple collision, basic bounce (no gyroscopes yet).

**Goal:** You have implemented **detection + response** once in a forgiving space.

---

## Phase 5 — Rigid body in 3D, empty space (1–2 weeks)

1. Represent one body: **position**, **orientation (quaternion)**, **linear velocity**, **angular velocity**, **mass**, **inertia tensor** (start with a **diagonal tensor in body space** for a symmetric top).
2. Apply **gravity** at the center of mass.
3. Integrate with **semi-implicit (symplectic) Euler:** update velocities from forces/torques, then update position and orientation.

**Goal:** A **tumbling** object in empty space that looks plausible (no stadium yet).

---

## Phase 6 — Graphics: enough to learn (1–2 weeks; can overlap Phase 5)

You do not need PBR. You need clarity.

1. **Shader + MVP** (model-view-projection), one simple mesh (cube or cylinder).
2. **Orbital camera** (mouse look is enough).
3. **Debug drawing:** velocity vector, spin axis, and later **contact normal** / tangent directions.

**Goal:** You can **see** bugs instead of guessing from numbers.

---

## Phase 7 — First Beyblade-style ingredient: simple stadium (1–2 weeks)

Prefer an **analytic** surface before imported meshes.

1. **Bowl:** e.g. **sphere segment** or **paraboloid** defined implicitly. Collision can start as **collider center vs surface** with a **radius offset** (sphere or small capsule proxy for the top).
2. **Contact:** one primary contact point is enough to start.
3. **Response:** separate **positional correction** along the **normal** from **impulses**; add **Coulomb friction** in the tangent plane (clamped tangential impulse).

**Goal:** Sliding, gripping, and orbiting in a bowl that you can **tune**.

---

## Phase 8 — Tuning and robustness (ongoing)

1. Tune **friction**, **restitution**, **angular damping**, and how aggressively you correct penetration.
2. Address **tunneling:** smaller `fixed_dt`, swept tests, or thicker proxies—learn which failure mode you hit first.
3. Add a **second top** only after one top is stable and debuggable.

**Goal:** Two tops interact without frequent explosive instabilities.

---

## Phase 9 — Optional: ship faster with a library

If solver and contact work drags motivation: use **Jolt Physics**, **Bullet**, or similar for rigid bodies and friction, and focus custom effort on **stadium representation, game rules, feel tuning, and presentation**. That is still a valid path to a Beyblade-style game.

---

## Milestone checklist (quick reference)

| Milestone | You know it works when… |
|-----------|-------------------------|
| Fixed timestep | Similar motion at 60 vs 144 FPS (within tolerance) |
| Free gyroscope in air | Plausible spin and precession under torque |
| One analytic stadium | Rolls/slides without obvious tunneling at your timestep |
| Friction tuning | Clearly “grippy” vs “slippery” behavior |
| Two tops | Collisions + friction without constant explosions |

---

## What makes Beyblade *feel* like Beyblade

- **Curved stadium contact** (not flat ground only).
- **Friction** that couples **linear** and **angular** motion at the contact patch.
- Later: **top–top** collisions, sleeping thresholds, and stable resting contact.

Advanced topics (after the checklist): sequential impulses, more contact points, broader phase for many objects, mesh stadiums (BVH), continuous collision detection.

---

## Study references (pick one primary track for a month)

- [Game Physics in a Weekend](https://gamephysicsweekend.github.io/) — compact, guided engine shape.
- Ian Millington, *Game Physics Engine Development* — matches “build my own engine” scope.
- Christer Ericson, *Real-Time Collision Detection* — when you leave simple spheres behind.

---

## Shortest summary

Learn **3D math** and **fixed timestep**, then **2D collision** for confidence, then **3D rigid bodies**, then **one analytic stadium + friction**, then **a second body**. Keep rendering at **debuggable minimum** until motion feels right.

If you choose **raw C++** vs **physics library** for collisions early, Phases 7–9 change the most; both are valid.
