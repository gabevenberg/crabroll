#set document(
  title: [Adapting LeibRamp to Integer Calculations],
  author: "Gabriel Venberg",
  date: datetime(
    year: 2025,
    month: 11,
    day: 23,
  ),
)

#show title: set align(center)

#title([
#context document.title\
#text(size: 0.75em, [
  #context document.author.first()\
  #context document.date.display()
])
])

= Introduction

A paper by Aryeh Eiderman @leibramp submits an efficient algorithm for real time stepper motor ramping where all expensive computations are precomputed,
leaving only multiplication and addition in the real-time loop.
However, Eidermans paper is explicitly designed for floating point arithmetic,
and does not work for integer arithmetic.
Eiderman notes that was originally designed for an IBM PC, which may have had a floating point coprocessor.
However, today, most stepper motors are controlled by microcontrollers, not full x86 machines.
Most microcontrollers do not have a floating point unit,
and so here we investigate a modification to allow Eidermans algorithm to work with integer arithmetic.

= Avoiding small numbers

Eidermans update formula is as follows:
$ p = p dot (1 + m dot p^2) $
Where:
$
  p & = "the delay between steps" \
  m & = cases(
        -a/F^2 "if accelerating",
        0 "if cruising",
        a/F^2 "if decelerating",
      ) \
  F & = "tick frequency "("Hz") \
  a & = "target acceleration "("steps"/"sec"^2)
$

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

= Remainder carrying

Unfortunately, the flooring after every division inherent in integer arithmetic reduces precision significantly,
and causes the acceleration curve to be asymmetrical with respect to the deceleration curve.
This can be fixed, however, by storing the remainder of each division and adding that remainder to the next iteration.
$
  p & =p plus.minus (p^3 + r)/m^(-1) \
  r & =(p^3 + r) mod m^(-1)
$

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

Adding remainder storage is straightforward with this enhancement,
though it requires 3 separate remainder variables to be stored:
$
  q^(-1) & =(m^(-1)+r_1)/p^2 \
       p & =p plus.minus (p+r_2)/q + (p+r_3)/q^2 \
     r_1 & =(m^(-1)+r_1) mod p^2 \
     r_2 & =(p+r_2) mod q \
     r_3 & =(p+r_3) mod q^2 \
$

Unlike Eidermans method, where this enhancement requires only one extra addition and one extra multiplication,
in the integer form it requires 2 extra divisions and an addition.
Due to the extra 2 divisions, and the extra space needed for the 2 extra remainders,
this was deemed not worth the extra precision in the authors usecase.

= Other variables

For convenience of the reader,
the following are the remaining variables needed to implement a linear ramping step planner.

Given as inputs or parameters:
$
    d & = "move distance" \
  v_0 & = "Inital speed "("steps"/"sec") \
    v & = "Max speed "("steps"/"sec") \
    F & = "tick frequency "("Hz") \
    a & = "target acceleration "("steps"/"sec"^2)
$

Precomputed before a move begins:
$
  p_1 & = F/sqrt(v_0^2 + 2a)                      && "delay period for inital step" \
  p_c & = F/v                                     && "delay period for cruise period steps" \
    S & = (v^2 - v_0^2)/(2a)                      && "distance needed for acceleration to "v \
  S_a & = cases(S "if" d>2S, ceil d/2 "if" d<=2S) && "actual distance needed for acceleration/decceleration" \
$

And finally, the _ideal_ formula, useful in unit tests and verification:
$ p = F/sqrt((F/p)^2 + 2a) $

$p$ should be clamped between $p_1$ and $p_c$.
When $p <= p_c$, the cruise phase should begin.
The deceleration phase should begin when the remaining steps in the move $<= S_a$.

#bibliography("works.bib")
