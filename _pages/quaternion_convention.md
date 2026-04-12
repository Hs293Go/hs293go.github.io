---
title: "Quaternions: A Common Convention"
permalink: /quaternion_convention/
---

# Quaternions: A Common Convention

Quaternions are a ubiquitous tool in computer graphics and robotics for
representing spatial rotations. However, there is a maddening array of different
conventions: Entire papers [^1] are written on the tradeoffs of different
conventions, and the Wikipedia page on Quaternions and Spatial Rotations
[dedicated a section](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Alternative_conventions),
to clarify the various conventions.

Instead of learning about different conventions, I propose picking a single
convention and sticking to it. Fortunately, major libraries in three programming
languages --- Python, C++, and rust --- have converged on a common quaternion
convention. This post introduces this common convention.

[^1]:
    Sommer, H., et al. "Why and how to avoid the flipped quaternion
    multiplication," in Aerospace, vol. 5, no. 3, pp. 72, 2018.

## Libraries

We considered the following libraries:

| Language | Framework | Class                              |
| -------- | --------- | ---------------------------------- |
| Python   | SciPy     | `scipy.spatial.transform.Rotation` |
| C++      | Eigen     | `Eigen::Quaternion`                |
| Rust     | nalgebra  | `nalgebra::UnitQuaternion`         |

## Representation

A quaternion consists of one real (scalar) component and three imaginary
(vector) components. The common convention represents quaternions as
four-dimensional vectors with the imaginary component **first** and the real
component **last** _by default_, resulting in a vector of the form:

$$
\mathbf{q} = \begin{bmatrix}
  \mathbf{q}_{xyz} \\
  q_w
\end{bmatrix}
$$

where $\mathbf{q}_{xyz}$ are the imaginary components, often treated as a
3-vector, and $q_w$ is the real component.

### Python (SciPy)

In SciPy, the `Rotation` class is created from lists or arrays representing
quaternions in the **scalar-last**/`[x, y, z, w]` order. . The `as_quat()`
method returns the quaternion in the same order:

```python
from scipy.spatial.transform import Rotation as R

rot = R.from_quat([x, y, z, w])
x, y, z, w = rot.as_quat()
```

### C++ (Eigen)

In Eigen, the `Quaternion` class can be constructed from a 4-vector, and it also
offers the `coeffs()` method to access the underlying vector representation.
These vectors are the **scalar-last**/`[x, y, z, w]` order.

```cpp
#include <Eigen/Geometry>

Eigen::Vector4d vec(x, y, z, w);
Eigen::Quaterniond q(vec);
const Eigen::Vector4d& underlying = q.coeffs();
assert(underlying.isApprox(vec));
```

> Eigen's vectors offer `x()`, `y()`, `z()`, and `w()` methods to access the
> individual components. Indeed, the `w()` method returns the 4th component, not
> the 1st.

### Rust (nalgebra)

In nalgebra, the `Quaternion` struct can be constructed from a 4-vector via the
`from_vector` function, and the `coords` method can be used to access the
underlying vector representation. These vectors are in the
**scalar-last**/`[x, y, z, w]` order.

```rust
use nalgebra::{Quaternion, Vector4};

let vec = Vector4::new(x, y, z, w);
let q = Quaternion::from_vector(vec);
assert_eq!(q.coords, vec);
```

> nalgebra vectors offer `i`, `j`, `k`, and `w` methods to access the individual
> components. Indeed, `w` returns the 4th component, not the 1st.

## Construction

While construction of quaternions from 4-vectors is consistent across the 3
languages and libraries, construction from individual components is much
messier.

### C++ (Eigen)

Eigen started this trend with its component-wise Quaternion constructor in the
**scalar-first** order/`[w, x, y, z]`. This is the opposite order of the
underlying vector representation.

```cpp
Eigen::Quaternion q(w, x, y, z);
Eigen::Quaternion p(Eigen::Vector4d(x, y, z, w));  // Opposite orders!

assert(q.coeffs().isApprox(p.coeffs()));
```

Fortunately, Eigen 5.0 added explicit factory functions to construct quaternions
from individual components in an explicitly specified order.

```cpp
Eigen::Quaternion q = Eigen::Quaterniond::FromCoeffsScalarFirst(w, x, y, z);
Eigen::Quaternion p = Eigen::Quaterniond::FromCoeffsScalarLast(x, y, z, w);

assert(q.coeffs().isApprox(p.coeffs()));
```

The mirroring explicit accessors for the underlying vector representation are
also available:

```cpp
auto q_vec = q.coeffs();             // [x, y, z, w]
auto p_vec = p.coeffsScalarLast();   // [x, y, z, w]
auto r_vec = p.coeffsScalarFirst();  // [w, x, y, z]
assert(q_vec.isApprox(p_vec));
```

### Rust (nalgebra)

nalgebra inherited the component-wise constructor from Eigen, and thus also uses
the **scalar-first** order/`[w, x, y, z]`.

```rust
use nalgebra::Quaternion;
let q = Quaternion::new(w, x, y, z);
```

## (\*) Python (SciPy)

Technically, SciPy does not have a component-wise constructor for `Rotation`
objects, but the `(from|as)_quat` methods in newer versions of SciPy offers a
`scalar_first` argument to opt into the **scalar-first** order/`[w, x, y, z]`
convention.

```python
from scipy.spatial.transform import Rotation as R

rot = R.from_quat([w, x, y, z], scalar_first=True)
x, y, z, w = rot.as_quat(scalar_first=True)
```

## Multiplication

`scipy.spatial.transform.Rotation`, `Eigen::Quaternion`, and
`nalgebra::UnitQuaternion` all overload the multiplication operator to perform
quaternion multiplication.

Multiplication of quaternions is consistent across the three libraries, and
follows the Hamilton product convention.

$$
\mathbf{p} \otimes \mathbf{q} = \begin{bmatrix}
  q_w \mathbf{1} + \mathbf{q}_{xyz}^\times & q_w \mathbf{q}_{xyz} \\
  -\mathbf{q}_{xyz}^\top & q_w
\end{bmatrix}\begin{bmatrix}
  p_{xyz} \\
  p_w
\end{bmatrix}
$$

### Python (SciPy)

```python
from scipy.spatial.transform import Rotation as R

rot1 = R.from_quat([x1, y1, z1, w1])
rot2 = R.from_quat([x2, y2, z2, w2])
rot_product = rot1 * rot2
```

### C++ (Eigen)

```cpp
#include <Eigen/Geometry>

Eigen::Quaterniond q1 =
    Eigen::Quaterniond::FromCoeffsScalarLast(x1, y1, z1, w1);
Eigen::Quaterniond q2 =
    Eigen::Quaterniond::FromCoeffsScalarLast(x2, y2, z2, w2);
Eigen::Quaterniond q_product = q1 * q2;
```

### Rust (nalgebra)

```rust
use nalgebra::Quaternion;
let q1 = Quaternion::new(w1, x1, y1, z1);
let q2 = Quaternion::new(w2, x2, y2, z2);
let q_product = q1 * q2;
```

> Rust distinguishes unit quaternions (`UnitQuaternion`) from general
> quaternions (`Quaternion`), but quaternion multiplication is identical for
> both types.

## Conversion to Rotation Matrices

The conversion from quaternions to rotation matrices is consistent across the
three libraries, and follows the standard formula:

$$
\mathbf{R} = \mathbf{1} + 2 q_w \mathbf{q}_{xyz}^\times + 2 \mathbf{q}_{xyz}^\times \mathbf{q}_{xyz}^\times = \begin{bmatrix}
  1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_z q_w) & 2(q_x q_z + q_y q_w) \\
  2(q_x q_y + q_z q_w) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_x q_w) \\
  2(q_x q_z - q_y q_w) & 2(q_y q_z + q_x q_w) & 1 - 2(q_x^2 + q_y^2)
\end{bmatrix}
$$

### Python (SciPy)

The `as_matrix()` method of `Rotation` objects returns the corresponding
rotation matrix.

```python
from scipy.spatial.transform import Rotation as R

rot = R.from_quat([x, y, z, w])
matrix = rot.as_matrix()
```

### C++ (Eigen)

The `toRotationMatrix()` method of `Quaternion` objects returns the
corresponding rotation matrix.

```cpp
#include <Eigen/Geometry>

Eigen::Quaterniond q = Eigen::Quaterniond::FromCoeffsScalarLast(x, y, z, w);
Eigen::Matrix3d matrix = q.toRotationMatrix();
```

### Rust (nalgebra)

Rust distinguishes unit quaternions (which represent rotations) from general
quaternions, and thus the `to_rotation_matrix()` method is only available for
`UnitQuaternion` objects.

```rust
use nalgebra::{Vector4, Quaternion, UnitQuaternion};

let q = Quaternion::from_vector(Vector4::new(x, y, z, w));
let q = UnitQuaternion::from_quaternion(q);
let matrix = q.to_rotation_matrix();
```

## Transformation of Vectors

The transformation of vectors by quaternions is consistent across the three
libraries, and is derived from the standard formula:

$$
\begin{bmatrix}
  \mathbf{v}' \\
  0
\end{bmatrix} = \mathbf{q} \otimes \begin{bmatrix}
  \mathbf{v} \\
  0
\end{bmatrix} \otimes \mathbf{q}^{-1}
$$

Albeit a more compact formula, derived from converting the quaternion to a
rotation matrix first then post-multiplying the vector, is almost always used in
practice:

$$
\mathbf{v}' = \mathbf{v} + 2 q_w \mathbf{q}_{xyz}^\times \mathbf{v} + 2 \mathbf{q}_{xyz}^\times\mathbf{q}_{xyz}^\times \mathbf{v}
$$

### Python (SciPy)

The `apply()` method of `Rotation` objects applies the rotation to a vector or
array of vectors.

```python
from scipy.spatial.transform import Rotation as R

rot = R.from_quat([x, y, z, w])
v = [vx, vy, vz]
v_rotated = rot.apply(v)
```

### C++ (Eigen)

The `operator*` of `Quaternion` objects applies the rotation to a vector.

```cpp
#include <Eigen/Geometry>

Eigen::Quaterniond q = Eigen::Quaterniond::FromCoeffsScalarLast(x, y, z, w);
Eigen::Vector3d v(vx, vy, vz);
Eigen::Vector3d v_rotated = q * v;
```

> This may appear to be abusing the multiplication operator, but it has
> precedent in C++ in OpenGL `glm::quat` quaternions

### Rust (nalgebra)

In nalgebra, the `transform_vector` method of `UnitQuaternion` objects applies
the rotation to a vector.

```rust
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

let q = Quaternion::from(Vector4::new(x, y, z, w));
let q = UnitQuaternion::from_quaternion(q);
let v = Vector3::new(vx, vy, vz);
let v_rotated = q.transform_vector(&v);
```
