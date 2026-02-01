# M1. Tangential Contact Basis Construction

> **Parent**: [math-formulation.md](math-formulation.md)
> **Depends on**: None (standalone derivation)
> **Required by**: [M2 (Friction Jacobian)](M2-friction-jacobian.md), [M8 (Numerical Examples)](M8-numerical-examples.md)

---

**Objective**: Given contact normal $\mathbf{n}$, construct deterministic, continuous orthonormal tangent basis $\{\mathbf{t}_1, \mathbf{t}_2\}$ such that $\{\mathbf{t}_1, \mathbf{t}_2, \mathbf{n}\}$ forms a right-handed orthonormal frame.

**Requirements**:
- **Orthonormality**: $\mathbf{t}_1 \cdot \mathbf{t}_2 = 0$, $\|\mathbf{t}_1\| = \|\mathbf{t}_2\| = 1$, $\mathbf{t}_i \cdot \mathbf{n} = 0$ for $i \in \{1, 2\}$
- **Determinism**: Same $\mathbf{n}$ always produces same $\{\mathbf{t}_1, \mathbf{t}_2\}$ (no random orientation)
- **Continuity**: Small change in $\mathbf{n}$ produces small change in $\mathbf{t}_1, \mathbf{t}_2$ (no discontinuities)
- **Degeneracy handling**: Works for all valid normals including alignment with coordinate axes

## Method: Building Orthonormal Basis from a Single Vector (Duff et al., 2017)

This method constructs a robust basis by selecting the coordinate axis most orthogonal to the input normal.

**Statement**:
$$
\mathbf{t}_1 =
\begin{cases}
\frac{1}{\sqrt{n_x^2 + n_z^2}} \begin{bmatrix} -n_z \\ 0 \\ n_x \end{bmatrix} & \text{if } |n_y| < |n_x| \text{ and } |n_y| < |n_z| \\[10pt]
\frac{1}{\sqrt{n_y^2 + n_z^2}} \begin{bmatrix} 0 \\ -n_z \\ n_y \end{bmatrix} & \text{if } |n_z| < |n_x| \text{ and } |n_z| < |n_y| \\[10pt]
\frac{1}{\sqrt{n_x^2 + n_y^2}} \begin{bmatrix} -n_y \\ n_x \\ 0 \end{bmatrix} & \text{otherwise}
\end{cases}
$$

$$
\mathbf{t}_2 = \mathbf{n} \times \mathbf{t}_1
$$

## Derivation

The key insight is to find a unit vector $\mathbf{t}_1$ perpendicular to $\mathbf{n}$ by crossing $\mathbf{n}$ with a coordinate axis that is not parallel to $\mathbf{n}$.

**Step 1**: Identify the coordinate axis most orthogonal to $\mathbf{n}$.

Let $\mathbf{n} = [n_x, n_y, n_z]^\top$. The coordinate axis $\mathbf{e}_i$ (where $i \in \{x, y, z\}$) most orthogonal to $\mathbf{n}$ is the one with smallest component magnitude $|n_i|$.

**Case 1**: If $|n_y| \leq |n_x|$ and $|n_y| \leq |n_z|$, then $\mathbf{e}_y = [0, 1, 0]^\top$ is most orthogonal.

**Step 2**: Compute unnormalized tangent vector by Gram-Schmidt orthogonalization:
$$
\tilde{\mathbf{t}}_1 = \mathbf{e}_y - (\mathbf{e}_y \cdot \mathbf{n})\mathbf{n} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} - n_y \begin{bmatrix} n_x \\ n_y \\ n_z \end{bmatrix} = \begin{bmatrix} -n_y n_x \\ 1 - n_y^2 \\ -n_y n_z \end{bmatrix}
$$

**Step 3**: Simplify using $\|\mathbf{n}\| = 1 \Rightarrow n_x^2 + n_y^2 + n_z^2 = 1$:
$$
1 - n_y^2 = n_x^2 + n_z^2
$$
$$
\tilde{\mathbf{t}}_1 = \begin{bmatrix} -n_y n_x \\ n_x^2 + n_z^2 \\ -n_y n_z \end{bmatrix}
$$

**Step 4**: Compute norm:
$$
\|\tilde{\mathbf{t}}_1\|^2 = n_y^2 n_x^2 + (n_x^2 + n_z^2)^2 + n_y^2 n_z^2
$$
Expanding:
$$
= n_y^2 n_x^2 + n_x^4 + 2n_x^2 n_z^2 + n_z^4 + n_y^2 n_z^2
$$
Factor:
$$
= n_x^2(n_y^2 + n_x^2 + 2n_z^2) + n_z^2(n_y^2 + n_z^2)
$$

However, a simpler approach recognizes that for the specific form we want, we can directly construct:
$$
\tilde{\mathbf{t}}_1 = \mathbf{e}_y \times \mathbf{n} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} n_x \\ n_y \\ n_z \end{bmatrix} = \begin{bmatrix} -n_z \\ 0 \\ n_x \end{bmatrix}
$$

Norm:
$$
\|\tilde{\mathbf{t}}_1\| = \sqrt{n_x^2 + n_z^2}
$$

Note that $n_x^2 + n_z^2 = 1 - n_y^2 > 0$ as long as $|n_y| < 1$ (which is guaranteed if $\mathbf{n}$ is not exactly aligned with $\mathbf{e}_y$).

**Step 5**: Normalize:
$$
\mathbf{t}_1 = \frac{\tilde{\mathbf{t}}_1}{\|\tilde{\mathbf{t}}_1\|} = \frac{1}{\sqrt{n_x^2 + n_z^2}} \begin{bmatrix} -n_z \\ 0 \\ n_x \end{bmatrix}
$$

**Step 6**: Compute second tangent:
$$
\mathbf{t}_2 = \mathbf{n} \times \mathbf{t}_1
$$

This is automatically unit-length and orthogonal to both $\mathbf{n}$ and $\mathbf{t}_1$ since $\|\mathbf{n}\| = \|\mathbf{t}_1\| = 1$ and $\mathbf{n} \perp \mathbf{t}_1$.

**Other cases** (where $|n_z|$ or $|n_x|$ is smallest) follow the same pattern using $\mathbf{e}_z$ or $\mathbf{e}_x$ respectively.

## Continuity Verification

As $\mathbf{n}$ varies continuously, the component magnitudes $|n_x|, |n_y|, |n_z|$ vary continuously. The branch selection (which component is smallest) only changes when two components become equal in magnitude (e.g., $|n_y| = |n_z|$). At these boundaries:
- The denominators $\sqrt{n_x^2 + n_z^2}$, etc., are never zero (since if $n_y = \pm 1$, then $n_x = n_z = 0$, which would select a different branch).
- The cross product $\mathbf{n} \times \mathbf{t}_1$ varies continuously.

Therefore, the basis varies continuously everywhere.

## Physical Interpretation

This construction selects the tangent direction $\mathbf{t}_1$ as the cross product of $\mathbf{n}$ with the most orthogonal coordinate axis. This ensures maximal separation from singularities (where $\mathbf{n}$ aligns exactly with a coordinate axis) and produces a deterministic, continuous tangent frame.
