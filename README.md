<div align="center">

# 🔢🦾 Prime-Frequency Neural Inverse Kinematics <br> vs. Halton–Prime Configuration-Space Planning

**Collision-free motion generation for planar redundant arms, swept from 5 to 30 DOF —<br>where the only thing that changes is the dimensionality, and every degree of freedom is indexed by a prime number.**

[![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://www.python.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.x-EE4C2C.svg)](https://pytorch.org/)
[![CUDA](https://img.shields.io/badge/CUDA-enabled-76B900.svg)](https://developer.nvidia.com/cuda-zone)
[![Reproducible](https://img.shields.io/badge/reproducible-deterministic-brightgreen.svg)]()
[![Hardware](https://img.shields.io/badge/tested%20on-NVIDIA%20Spark-76B900.svg)]()

*L.A. Muñoz — Computational Robotics Lab, Tec de Monterrey — July 2026*

</div>

---

## 📦 What's in this repo

Two artifacts that are **two forms of the same work**:

| File | What it is |
|------|------------|
| 📄 [`PrimeFrequency_Neural_Inverse_Kinematics_versus_HaltonPrime_ConfigurationSpace_Planning_from_5_to_30_DOF.pdf`](./PrimeFrequency_Neural_Inverse_Kinematics_versus_HaltonPrime_ConfigurationSpace_Planning_from_5_to_30_DOF.pdf) | **The paper** — motivation, full closed-form equations (kinematics, collision model, losses, planner), the *"why primes"* argument, and discussion of results. |
| 📓 [`prime_ik_multi_dof_experiments.ipynb`](./prime_ik_multi_dof_experiments.ipynb) | **The notebook** — the *executable form* of the paper. Every equation in the PDF is implemented here, and every figure, animation, and CSV is emitted by running it. |

> The paper tells you *why*; the notebook lets you *reproduce it, byte for byte*. Both prime constructions are fully deterministic — same samples, same frequencies, on any hardware.

---

## 💡 The idea in one paragraph

Growing a robot arm from *N* to *N*+1 joints normally forces you to re-engineer its representation: new frequency schedules for learned encodings, re-drawn projections, re-balanced samplers. This work develops a single design principle that makes scaling free: **index every degree of freedom by a distinct prime number**. Because primes are pairwise coprime and infinitely extensible:

- 🎵 **Sinusoidal input channels at prime frequencies never share harmonics** — each joint gets a spectrally private channel, unlike powers-of-2 positional encodings where every frequency divides all higher ones.
- 🎲 **Halton samples with one prime base per joint stay decorrelated** across dimensions.

Adding a joint means consuming the next prime. Nothing already learned or assigned is touched.

---

## ⚔️ The two methods compared

### Method A — Neural-Prime *(learning-based)*

A GPU-parallel **population of 64 IK policies** (3-layer GELU MLPs, batched as one `bmm` per layer), whose inputs are quantized through sin/cos channels at the *k*-th prime frequency for joint *k*. Trajectories are **continuous by construction** (bounded 8° increments). Four robustness mechanisms make it survive 20–30 DOF:

1. 🧘 Near-zero output initialization (*"hold still"*)
2. 🚧 Absolute self-collision barrier active from epoch 0 — crossings must be *prevented*, never repaired
3. ✅ Population-wide feasibility selection
4. ✨ Feasible-snapshot polishing

### Method B — C-space Halton *(classical)*

**Damped least-squares IK** to obtain a goal configuration, then **RRT-Connect** in joint space where random sampling is replaced by the deterministic, low-discrepancy **Halton sequence with one distinct prime base per joint** — benchmarked head-to-head against seeded uniform sampling at every DOF with identical budgets.

### 🔬 The protocol

Total reach is fixed at **L = 10** (link length *L/N*); the start pose, goal **(6, −6)**, and three circular obstacles are **identical** across *N* ∈ {5, 10, 15, 20, 25, 30}. Task difficulty is constant; only the search space explodes. The free fraction of the configuration space collapses roughly **exponentially** with *N* — driven by the quadratically growing family of self-collision pairs — and that collapse is *measured, not assumed*.

---

## 📊 Headline results

| | Finding |
|---|---|
| 🏃 | **Method B stays cheap and precise.** DLS + RRT-Connect is orders of magnitude cheaper than population training in tokens and wall time, and pins the endpoint to very low final error across the whole sweep. The Halton–prime sampler's advantage over uniform (fewer iterations, fewer collision queries) appears and *widens* at higher DOF — exactly where coverage quality matters most. |
| 🧠 | **Method A tracks the whole path and amortizes.** It optimizes Cartesian tracking along the entire trajectory (not just the endpoint), and its training cost amortizes when the policy is reused across queries. |
| ⚠️ | **Honest caveat: Neural-Prime degrades past ~20 DOF** (collisions appear; final and tracking errors jump — see Figure 1 of the paper). Two effects compound: large primes mean large frequencies (the 20th prime is already 71), which ill-conditions the encoding at high *N*; and plain (unscrambled) Halton develops correlated stripe patterns in high-index dimensions. The suite deliberately uses the plain constructions and reports the uniform baseline at every DOF, so any degradation is **visible rather than hidden**. Scrambled/leaped Halton variants and per-magnitude frequency scaling are the natural mitigations, discussed in the paper. |

---

## 📈 What the notebook produces

Running the notebook end-to-end executes all six experiments and emits:

- 🖼️ **`figures/`** — workspace overview, C-space free-fraction collapse, per-DOF training convergence, joint profiles, per-step continuity, and the final six-panel comparison (Figure 1 of the paper).
- 🎞️ **`animations/`** — one side-by-side GIF per DOF (Method A vs. Method B executing the task).
- 📋 **Summary table + machine-readable CSV** with every metric: final error, tracking error, continuity (max |Δq|), min external/internal clearances, dense-validation verdict, wall time, hardware-agnostic token counts, and the Halton-vs-uniform sampler comparison.

### 🗺️ Notebook structure (mirrors the paper)

```text
0.  Setup, CUDA device, adaptive budgets (QUICK_TEST mode available)
1.  Shared scenario + parametric N-DOF arm (FK, Jacobian, collision model)   → Paper §2
2.  Shared obstacle-free Cartesian reference path (optimized once)           → Paper §2.3
3.  C-space free-fraction estimation, Halton-prime vs uniform (GPU-batched)  → Paper §3.2, §6.1
4.  Method A: population of prime-encoded IK policies + polishing            → Paper §3.1, §4
5.  Method B: DLS + RRT-Connect with prime-base Halton sampling              → Paper §5
6.  Running the six experiments (5–30 DOF)
7.  Per-experiment visualizations
8.  GIF animations
9.  Final comparative analysis + CSV                                         → Paper §7
10. Conclusions                                                              → Paper §8
```

---

## 🚀 Requirements & how to run

```bash
pip install torch numpy matplotlib
jupyter notebook prime_ik_multi_dof_experiments.ipynb
```

| Mode | Details |
|------|---------|
| ⚡ **GPU (recommended)** | Developed and run on an **NVIDIA Spark**. On CUDA the suite scales up automatically: population 64, hidden width 256, 1500+ epochs, 60,000 batched C-space samples. Everything is batched — forward kinematics, all clearances, the exact free-configuration predicate, and the entire policy population run in single GPU calls. |
| 🐢 **CPU** | Works too, with automatically reduced budgets — same code, just slower and smaller. |
| 🧪 **Smoke test** | `QUICK_TEST=1 jupyter notebook ...` runs a tiny end-to-end pass (DOF ∈ {5, 30}, reduced budgets) to verify the whole pipeline in minutes. |

> 🔁 **Reproducibility is by construction**: fixed seeds, deterministic Halton tables, deterministic prime frequencies. Two runs of the suite draw *exactly* the same samples.

---

## 🗂️ Repository map

```text
.
├── README.md
├── PrimeFrequency_Neural_Inverse_Kinematics_versus_HaltonPrime_...pdf   # the paper
├── prime_ik_multi_dof_experiments.ipynb                                 # the executable paper
├── figures/       # emitted by the notebook
└── animations/    # emitted by the notebook (one GIF per DOF)
```

---

## 📝 Citation

```bibtex
@techreport{munoz2026primeik,
  title       = {Prime-Frequency Neural Inverse Kinematics versus Halton--Prime
                 Configuration-Space Planning from 5 to 30 DOF},
  author      = {Mu{\~n}oz, L.A.},
  institution = {Computational Robotics Lab, Tec de Monterrey},
  year        = {2026},
  month       = {July}
}
```

## 📚 Key references

Halton (1960) on quasi-random sequences · Niederreiter (1992) on quasi-Monte Carlo · Kuffner & LaValle (2000) on RRT-Connect · Nakamura & Hanafusa (1986) and Wampler (1986) on damped least-squares IK · Mildenhall et al. (2020) and Tancik et al. (2020) on Fourier-feature positional encodings — the powers-of-2 schedules to which the prime schedule is the antithesis. Full list in the paper.

---

<div align="center">

*"Increasing the number of degrees of freedom stops being a representational event at all: it is the consumption of the next element of a sequence that number theory finished preparing long ago."* — §7

</div>

---

*"Increasing the number of degrees of freedom stops being a representational event at all: it is the consumption of the next element of a sequence that number theory finished preparing long ago."* — §7
