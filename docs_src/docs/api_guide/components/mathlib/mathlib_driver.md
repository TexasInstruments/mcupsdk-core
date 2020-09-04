# MATHLIB {#MATHLIB_DRIVER}

[TOC]

## Introduction

Trigonometric functions are commonly used in real-time control applications, particularly within the inner loops of control algorithms, where speed and accuracy is essential. The performance of trigonometric functions is a key careabout for designers of these systems as it can have a significant impact on the overall performance of the system. Until recently, trignometric functions based on lookup tables were considered faster than the polynomial-based methods; however, with the inclusion of floating-point units (FPUs) and faster clock speeds, polynomial-based approximations have gained favor. TI has developed C functions of the most commonly used trigonometric functions using these polynomial-based methods and has optimized them for TI's Arm®-based microcontrollers (MCUs) and microprocessors (MPUs).

## Features Supported

Supports Below trignometric functions

- Sine
- Cosine
- Sine + Cosine
- Arcsine
- Arccosine
- Arctangent
- Arctangent2

## Features Not Supported

NA

## Additional References {#MATHLIB_ADDITIONAL_REFERENCES}

<a href="https://www.ti.com/lit/pdf/sprad27">App Note: Optimized Trigonometric Functions on TI Arm Cores</a>

## API

\ref DRV_MATHLIB_MODULE
