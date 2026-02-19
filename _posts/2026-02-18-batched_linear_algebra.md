---
title: Batched linear algebra in Python, Torch, and JAX
permalink: /batched-linear-algebra/
---

Sometimes, when your vibe coding agent produce elegant-looking batchwise linear
algebra code, **DO NOT TRUST IT**. Fortunately, the behavior of batched linear
algebra operations in `numpy`, `torch`, and `jax` is consistent, if not well
documented. Learning a few basic recipes can help you write the correct code
yourself unaided by AI.

Numpy's documentation for `np.matmul` states:

> If either argument is N-D, N > 2, it is treated as a stack of matrices
> residing in the last two indexes and broadcast accordingly.

Based on this, we can perform many linear algebra operations over batches
without explicitly writing a loop or even `vmap`.

- Batched matrix-matrix multiplication (GEMM):

  This is remarkably easy, because as long as all leading dimensions until the
  last two are broadcastible, the `@` operator will just work:

  ```python
  l = torch.rand(b1, b2, m, n)  # a (b1 x b2) batch of (m x n) matrices
  r = torch.rand(b1, b2, n, p)  # a (b1 x b2) batch of (n x p) matrices
  out = l @ r  # a (b1 x b2) batch of (m x p) matrices
  ```

- Batched matrix-vector multiplication (GEMV):

  In a non-batched context, the `@` operator will automatically pad trailing
  dimensions of size 1, so we can multiply a 2D array with a 1D array:

  ```python
  l = torch.rand(m, n)  # a (m x n) matrix
  r = torch.rand(n)  # a (n,) array, treated as a (n x 1) matrix
  out = l @ r  # a (m,) array (the trailing dimension of size 1 is squeezed out)
  ```

  In batch mode, the `@` operator will **NOT** automatically pad trailing
  dimensions of size 1, so we must explicitly add a trailing dimension:

  ```python
  l = torch.rand(b1, b2, m, n)  # a (b1 x b2) batch of (m x n) matrices
  r = torch.rand(b1, b2, n)  # a (b1 x b2) batch of (n,) arrays
  out = l @ r[..., None]  # a (b1 x b2) batch of (m x 1) matrices
  out = out.squeeze(-1)  # a (b1 x b2) batch of (m,) arrays

  out = (l @ r[..., None]).squeeze(-1)  # a (b1 x b2) batch of (m,) arrays in one line
  ```

> **NOTE**: Indexing with `None` is a common idiom to add a new axis of size 1
> at the indexed position. For example:
>
> - `r[..., None]` adds a new axis at the end of the shape of `r`, e.g., turning
>   it from `(m, n, p)` to `(m, n, p, 1)`
> - `r[..., None, :]` adds a new axis before the last axis of `r`, e.g., turning
>   it from `(m, n, p)` to `(m, n, 1, p)`
> - `r[None, ...]` adds a new axis at the beginning of the shape of `r`, e.g.,
>   turning it from `(m, n, p)` to `(1, m, n, p)`
>
> The names of methods to add new axes vary between libraries: What was
> `expand_dims` in NumPy is `unsqueeze` in PyTorch. While the named methods let
> us to add axes at arbitrary numbered axes, we don't need that much flexibility
> in this context, so we stick to the `None` indexing idiom.

- Batched vector-matrix multiplication:

  This is just the reverse of the previous case, so we add a new axis in the
  penultimate position of the LHS:

  ```python
  l = torch.rand(b1, b2, m)  # a (b1 x b2) batch of (m,) arrays
  r = torch.rand(b1, b2, m, n)  # a (b1 x b2) batch of (m x n) matrices

  out = l[..., None, :] @ r  # a (b1 x b2) batch of (1 x n) matrices
  out = out.squeeze(-2)  # a (b1 x b2) batch of (n,) arrays

  out = (l[..., None] @ r).squeeze(-2)  # a (b1 x b2) batch of (n,) arrays in one line
  ```

- Batched vector-vector dot product:

  If you think like a mathematician, you may write:

  ```python
  l = torch.rand(b1, b2, n)  # a (b1 x b2) batch of (n,) arrays
  r = torch.rand(b1, b2, n)  # a (b1 x b2) batch of (n,) arrays

  # Expand the LHS to (b1 x b2 x 1 x n) and the RHS to (b1 x b2 x n x 1),
  # then multiply to get a (b1 x b2 x 1 x 1) array, and finally squeeze out the
  # last two dimensions to get a (b1 x b2) array
  out = (l[..., None, :] @ r[..., None]).squeeze()
  ```

  However, if you can identify the dot product's nature as a "map-reduce"
  operation (sum of elementwise products), you could write:

  ```python
  (l * r).sum(-1)  # a (b1 x b2) batch of scalars
  ```

  which is more performant (about 2x on a i6-12600K CPU).

  ```
  [ins] In [2]: import torch

  [ins] In [3]: a = torch.rand(80, 20, 64)

  [ins] In [4]: b = torch.rand(80, 20, 64)

  [ins] In [5]: %timeit (a[..., None, :] @ b[..., None]).squeeze()
  40.6 μs ± 85.2 ns per loop (mean ± std. dev. of 7 runs, 10,000 loops each)

  [ins] In [6]: %timeit (a * b).sum(-1)
  22.3 μs ± 6.17 μs per loop (mean ± std. dev. of 7 runs, 100,000 loops each)

  [ins] In [7]: assert torch.allclose((a[..., None, :] @ b[..., None]).squeeze(), (a * b).sum(-1))
  ```

- Batched outer product:

  No tricks here, just add the appropriate new axes to get the desired shapes
  for matrix multiplication:

  ```python
  l = torch.rand(b1, b2, m)  # a (b1 x b2) batch of (m,) arrays
  r = torch.rand(b1, b2, n)  # a (b1 x b2) batch of (n,) arrays

  out = l[..., None] @ r[..., None, :]  # a (b1 x b2) batch of (m x n) matrices
  ```

- Batched quadratic form:

  It is notable that regardless if you have a different matrix for each bath or
  a shared matrix across the batch, the same code works:

  ```python
  l = torch.rand(b1, b2, n)  # a (b1 x b2) batch of (n,) arrays
  r = torch.rand(b1, b2, n)  # a (b1 x b2) batch of (n,) arrays
  A1 = torch.rand(b1, b2, n, n)  # a (b1 x b2) batch of (n x n) matrices
  A2 = torch.rand(n, n)  # a single (n x n) matrix shared across the batch

  # Both results in a (b1 x b2 x 1 x 1) array, squeeze to (b1 x b2) if needed
  out1 = l[..., None, :] @ A1 @ l[..., None]
  # A2 is treated as (1 x 1 x n x n) and broadcast across the batch
  out2 = l[..., None, :] @ A2 @ l[..., None]
  ```

  This is because numpy and its cousins automatically prepend leading dimensions
  of size 1 to the operands as needed to make their shapes compatible for matrix
  multiplication.
