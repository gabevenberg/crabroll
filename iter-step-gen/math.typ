= Introduction

The paper http://hwml.com/LeibRamp.pdf by Aryeh Eiderman submits an efficient algorithm for real time stepper motor ramping where all expensive computations are precomputed,
leaving only multiplication and addition in the real-time loop.
However, Eidermans paper is explicitly designed for floating point arithmetic,
and does not work for integer arithmetic.
Eiderman notes that was originally designed for an IBM PC, which may have had a floating point coprocessor.
However, today, most stepper motors are controlled by microcontrollers, not full x86 machines.
Most microcontrollers do not have a floating point unit,
and so here we investigate a modification to allow Eidermans algorithm to work with integer arithmetic.

= Modifications

Eidermans update formula is as follows:
$ p = p dot (1 + m dot p^2) $
Where:
$ p = "the delay between steps" $
$
  m = cases(
    -a/F^2 "if accelerating",
    0 "if cruising",
    a/F^2 "if decelerating",
  )
$
$ F = "the tick frequency" $
$ a = "the acceleration in steps/sec"^2 $

This algorithm works fine for floating point values,
and indeed, the paper calls out that this algorithm is designed for them.
However, for integer math, natively transcribing the above algorithm into code results in several issues:

- $m$ is almost always 0, as $F^2$ is very large (for some micrcontrollers, it is in-fact dangerously close to $2^64$)
- $(1+m dot p^2)$ is intended to always be between 0 and 2, usually around 1.
  In integer math, this means it is always 0 or 1, resulting in no motion or no acceleration.

However, we can do a few transformations to avoid small numbers in intermediate calculations,
making the fractional part much less significant.

Firstly, instead of storing the (most likely precomputed) $m=a/F^2$, we can store its inverse,
$m^(-1)=F^2/a$. This will be a very large number rather than a very small number,
avoiding truncation to zero.
Due to this transformation, we now divide by $m^(-1)$.
The update formula becomes:
$ p=p dot (1+p^2/m^(-1)) $

Secondly, we can change the grouping of the final calculation.
Where $(1+m dot p^2)$ is $tilde.equiv 1$, both $m$ and $p^2$ are relatively large.
We can use this to distribute $p$, causing the intermediate calculations to avoid small numbers, like so:
$ p=p+p^3/m^(-1) $

Finally, if we are also using unsigned integers, during acceleration we can, instead of negating $m^(-1)$,
we can subtract $p$ from $p^3/m^(-1)$, making the update function:
$ p=p plus.minus p^3/m^(-1) $

= Remainder storage

Unfortunately, the flooring after every division inherent in integer arithmetic reduces precision significantly,
and causes the acceleration curve to be asymmetrical with respect to the deceleration curve.
This can be fixed, however, by storing the remainder of each division and adding that remainder to the next iteration.
$ p=p plus.minus (p^3 + r)/m^(-1) $
$ r=(p^3 + r) mod m^(-1) $

= Modifying the optional enhancement

Eiderman posits an optional precision enhancement using a couple extra computations to increase the accuracy of the algorithm:
$ p=p dot (1+q+q^2) $
where $q = m dot p^2$.

We can apply similar transformations to this. As we have already calculated $m^(-1)$, we can redefine $q$ as:
$ q=p^2/m^(-1) $

and distribute $p$:
$ p=p plus.minus p q + p q^2 $

Unfortunately, $q$ is also very close to 0, so we instead calculate the inverse,
$q^(-1) = m^(-1)/p^2$.

and divide rather than multiply:
$ p=p plus.minus p/q + p/q^2 $

Applying remainder storage to this enhancement is left as an exercise to the reader.
