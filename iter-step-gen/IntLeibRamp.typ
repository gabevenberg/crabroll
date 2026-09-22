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

#set math.equation(numbering: "(1)", supplement: [Eq.])

#title([
  #context document.title\
  #text(size: 0.75em, [
    #context document.author.first()\
    #context document.date.display()
  ])
])

= Introduction

A paper by Aryeh Eiderman @leibramp submits an efficient algorithm for real time stepper motor ramping where all expensive computations are precomputed,
leaving only multiplication and addition in the real time loop.
However, Eiderman's paper is explicitly designed for floating point arithmetic,
and does not work for integer arithmetic.
Eiderman notes that was originally designed for an IBM PC, which may have had a floating point coprocessor.
However, today most stepper motors are controlled by microcontrollers and not full x86 machines.
Many microcontrollers do not have a floating point unit and so here we investigate a modification to allow Eiderman's algorithm to work with integer arithmetic.

This modification was developed for `iter-step-gen`, a Rust-based asynchronous stepper motor step planner and driver written out of spite to control the author's window blinds with an esp32c3.

= Original formula

With the following inputs,
$
    d & = "move distance" \
  v_0 & = "Initial speed "("steps"/"sec") \
    v & = "Max speed "("steps"/"sec") \
    F & = "tick frequency "("Hz") \
    a & = "target acceleration "("steps"/"sec"^2)
$ <inputs>

Eidermans update formula is as follows:
$ p = p(1 + m p^2) $ <original_formula>
Where:
$
  p & = "the delay between steps" \
  m & = cases(
        -a/F^2 "if accelerating",
        0 "if cruising",
        a/F^2 "if decelerating",
      ) \
$ <original_formula_definittions>

= Avoiding small numbers

LeibRamp works fine for floating point values,
and indeed, the paper calls out that this algorithm is designed for them.
However, for integer math, natively transcribing the above algorithm into code results in several issues:

- $m$ is almost always 0, as $F^2$ is very large (for some microcontrollers, it is in-fact dangerously close to $2^64$)
- $(1+m p^2)$ is intended to always be between 0 and 2, usually around 1.
  In integer math, this means it is always 0 or 1, resulting in no motion or no acceleration.

However, we can do a few transformations to avoid small numbers in intermediate calculations,
making the fractional part much less significant.

Firstly, instead of storing the (most likely precomputed) $m=a/F^2$, we can store its inverse,
$m^(-1)=F^2/a$. This will be a very large number rather than a very small number,
avoiding truncation to zero.
Due to this transformation, we now divide by $m^(-1)$ in @original_formula.
The update formula becomes:
$ p=p(1+p^2/m^(-1)) $ <store_inverse>

Secondly, we can change the grouping of the final calculation.
Where $(1+m p^2)$ is $tilde.equiv 1$, both $m$ and $p^2$ are relatively large.
We can use this to distribute $p$ in @store_inverse,
causing the intermediate calculations to avoid small numbers, like so:
$ p=p+p^3/m^(-1) $ <distribute>

Finally, if we are also using unsigned integers,
during acceleration we can,
instead of negating $m^(-1)$ in @distribute,
we can subtract $p$ from $p^3/m^(-1)$, making the update function:
$ p=p plus.minus p^3/m^(-1) $ <plus_minus>

= Remainder carrying

Unfortunately, the flooring after every division inherent in integer arithmetic reduces precision significantly,
and causes the acceleration curve to be asymmetrical with respect to the deceleration curve.
This can be mostly fixed, however, by storing the remainder of each division and adding that remainder to the next iteration, turning
@plus_minus into the following pair of equations:
$
  p & =p plus.minus (p^3 + r)/m^(-1) \
  r & =(p^3 + r) mod m^(-1)
$ <remainder_carrying>

= Modifying the optional enhancement

Eiderman posits an optional precision enhancement using a couple extra computations to increase the accuracy of the algorithm:
$ p=p(1+q+q^2) $ <original_enhancement>
where $q = m p^2$.

We can apply similar transformations to this. As we have already calculated $m^(-1)$, we can redefine $q$ as:
$ q=p^2/m^(-1) $ <redefined_q>

And distribute $p$ in @original_enhancement:
$ p=p plus.minus p q + p q^2 $ <distribute_p>

Unfortunately, $q$ is also very close to 0, so we instead calculate the inverse,
$q^(-1) = m^(-1)/p^2$.

And divide rather than multiply in @distribute_p:
$ p=p plus.minus p/q + p/q^2 $ <div_q>

Adding remainder storage is straightforward with this enhancement,
though it requires 3 separate remainder variables to be stored:
$
  q^(-1) & =(m^(-1)+r_1)/p^2 \
       p & =p plus.minus (p+r_2)/q + (p+r_3)/q^2 \
     r_1 & =(m^(-1)+r_1) mod p^2 \
     r_2 & =(p+r_2) mod q \
     r_3 & =(p+r_3) mod q^2 \
$ <enhancement_remainder_storage>

Unlike Eidermans method, where this enhancement requires only one extra addition and one extra multiplication,
in the integer form it requires 2 extra divisions and an addition.
Due to the extra 2 divisions, and the extra space needed for the 2 extra remainders,
this was deemed not worth the extra precision in the authors use case.

= Measuring the ramps <measuring_the_ramps>

Remainder carrying removes most of the asymmetry between the two ramps, but not all of it.
Two sources of error survive it, both leading to a shorter acceleration and longer deceleration than ideal.

First, @plus_minus is a first degree approximation.
Expanding @ideal_formula gives $p(1 plus.minus q + 3/2 q^2 plus.minus ...)$, where $q=m p^2$,
with the same sign convention as @plus_minus,
so dropping everything past the first term leaves $p$ on the fast side of the ideal curve,
whether accelerating or decelerating.

Second, $p$ is a whole number of ticks, and every update is calculated from $p^3$.
The remainder carries the fraction lost by the division,
but nothing carries the fraction lost by $p$ itself,
leading to $p$ being consistently short.

While both errors are tiny per step, neither of them cancels over the length of a ramp.
Sitting consistently on the fast side means acceleration reaches $p_c$ in fewer updates than $S$,
and deceleration needs more than $S$ to climb back to $p_1$.
With the parameters used in the authors use case
($v=255 "steps"/"sec"$, $v_0=50 "steps"/"sec"$, $a=64 "steps"/"sec"^2$ and $F=1 "MHz"$),
$S$ works out to 488.5 steps,
while the implemented ramp accelerates in 485 updates and decelerates in 490.

This shows up at the end of a move.
The deceleration phase ends when the target position is reached,
wherever $p$ has got to by then,
and the motor is expected to stop dead from that speed.
A deceleration phase sized with $S$ therefore finishes a couple of steps short of $p_1$,
and the last step of the move asks for more than $a$.
A fudge factor on top of $S$ will fix that for only one set of parameters,
as the size of the error depends on all of $v$, $v_0$, $a$ and $F$.

Rather than look for a closed form for the error,
we can measure both ramps directly,
by running @remainder_carrying over them once when the planner is constructed:
$
  S_a & = "updates to get from" p_1 "to" p_c && quad "acceleration ramp length" \
  S_d & = "updates to get from" p_c "to" p_1 && quad "deceleration ramp length" \
    S_l & = S_d - S_a                          && quad "ramp lag" \
$ <measured_ramps>

Both counts should be capped at the length of the axis.
A ramp that does not fit on the axis can never be run to completion anyway,
and the cap keeps the two loops finite for a badly configured stepper.

= Implementation considerations

For convenience of the reader,
the following are the remaining variables needed to implement a linear ramping step planner.
$
  p_1 & = F/sqrt(v_0^2 + 2a)            && "delay period for initial step" \
  p_c & = F/v                           && "delay period for cruise period steps" \
    S & = (v^2 - v_0^2)/(2a)            && "ideal distance needed for acceleration to "v \
  S_m & = min(S_d, ceil((d + S_l)/2))     && "steps of a "d" step move to spend decelerating" \
$ <implementation_vars>

$S$ is not used directly, only $S_a$, $S_d$ and $S_l$ from @measured_ramps.
A move that is long enough to reach $v$ needs $S_a$ updates to accelerate and $S_d$ updates to come back down.
A move shorter than that splits its steps between the two ramps instead,
and the split is not even, as deceleration wants $S_l$ more steps than acceleration.
The $min$ covers both cases.

A move can be split into 3 parts, the acceleration phase, the cruise phase, and the deceleration phase.
During the acceleration phase, which lasts until $p <= p_c$, the $plus.minus$ is a subtraction.
During the cruise phase, which lasts until the remaining steps in the move $<=S_m$, $p$ should be held constant at $p_c$.
During the deceleration phase, which lasts until the target position is reached, the $plus.minus$ is an addition.

Finally, the _ideal_ formula, useful in unit tests and verification, is:
$ p = F/sqrt((F/p)^2 + 2a) $ <ideal_formula>

#bibliography("works.bib")
