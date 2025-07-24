import numpy as np
from numpy import log10, exp, sqrt, pi, cos, sin, tan, arccos, arcsin, arctan, arctan2, sinh, cosh, tanh, arcsinh, arccosh, arctanh

from copy import deepcopy
from numpy.linalg import norm
from matplotlib import pyplot as plt

def MaxNormIntegrate(y0 : float, y1 : float, epsilon : float) -> float:
    # Error criterion based only on de/dV = -P
    # du/dV = -rho*c comes along for the ride
    # avoids small time steps at the initial step if e(0) = 0 or u(0) = 0
    diff = y0 - y1
    abs_err = np.sum(np.sqrt(diff*diff), axis=0)        # norm(y0[0] - y1[0])
    normy0 = np.sum(np.sqrt(y0*y0), axis=0)             # norm(y0[0])
    normy1 = np.sum(np.sqrt(y1*y1), axis=0)             # norm(y1[0])
    rel_err = abs_err / (normy0 + normy1 + epsilon)     # norm(y0[0] - y1[0]) / (norm(y0[0]) + norm(y1[0]) + epsilon)
    return np.min([rel_err, abs_err], axis = 0) / epsilon

def MaxNormODE(y0 : float, y1 : float, epsilon : float) -> float:
    # Error criterion based only on de/dV = -P
    # du/dV = -rho*c comes along for the ride
    # avoids small time steps at the initial step if e(0) = 0 or u(0) = 0
    err = max(abs(y0 - y1) / ((abs(y0) + abs(y1) + epsilon)*epsilon))
    return err

def MaxNormODEp1(y0 : float, y1 : float, epsilon : float) -> float:
    # Error criterion based only on de/dV = -P
    # du/dV = -rho*c comes along for the ride
    # avoids small time steps at the initial step if e(0) = 0 or u(0) = 0
    err = max(abs(y0 - y1) / ((abs(y0) + abs(y1) + 1)*epsilon))
    return err

def myfsolve(func : callable, bottomBound : float, topBound : float, val : float = 0, 
             valBottom: float = None, valTop: float = None, rel_tol = 1e-10):
    """
    Solve for a value v such that func(v) - val = 0 by binary search.
    func is a function that takes a single value as input and returns a single value.
    bottomBound and topBound are the lower and upper bounds of the search, respectively.
    val is the target value. Default is 0.
    """
    err = 0
    v0 = bottomBound
    if valBottom is None:
        valBottom = func(v0)
    valBottom-=val
    v1 = topBound
    if valTop is None:
        valTop = func(v1)
    valTop-=val

    if valBottom * valTop > 0 and valBottom < valTop:
        v0 = topBound
        valBottom = func(v0) - val
        v1 = bottomBound
        valTop = func(v1) - val

    # binary search
    tol = log10(rel_tol)
    while log10(abs((v0 - v1) / v1)) > tol:
        if valBottom * valTop < 0:
            vt = (v0 + v1) / 2
            valc = func(vt)
            if (valc - val) * valBottom > 0:
                valBottom = valc - val
                v0 = vt
            else:
                valTop = valc - val
                v1 = vt
        else:
            vt = v1 - valTop * (v1 - v0) / (valTop - valBottom)
            v0 = v1
            valBottom = valTop
            v1 = vt
            valTop = func(v1) - val
            valc = valTop

    xerror = log10(abs((valc - val) / valc))
    ferror = log10(abs((v0 - v1) / v1))
    if abs(val) > 0 and log10(abs((valc - val) / valc)) > - 2:
        err = 1
    res = vt
    return res, err, [xerror, ferror]

##################################################################################################
## advanceRK4
def advanceRK4(func, tspan, f0, dt_prev, f0_prev = None, numOfSteps : int = 1) -> list:
    tim = tspan  
    tim = np.linspace(tspan[0], tspan[ - 1], numOfSteps + 1)  
    
    h = tim[1] - tim[0]  
    y = f0  

    k_1 = func(tim[0],           deepcopy(y),         dt_prev, f0_prev    ) 
    k_2 = func(tim[0] + 0.5 * h, y + 0.5 * h * (k_1), 0.5 * h, deepcopy(y))
    k_3 = func(tim[0] + 0.5 * h, y + 0.5 * h * (k_2), 0.5 * h, deepcopy(y))
    k_4 = func(tim[0] + h,       y + h * (k_3),             h, deepcopy(y))
    dy = (1. / 6.) * (k_1 + 2 * k_2 + 2 * k_3 + k_4) * h    # main equation
    # dy2 = 1/3*(k_1*h/2 + k_3*h) + 2.0/3.0*k_2*h/2 + k_4*h/6 # = k_1*h/6 + k_2*h/3 + k_3*h/3 + k_4*h/6 = (k_1 + 2*k_2 + 2*k_3 + k_4)*h/6
    y = y + dy
    
    return tim, y, dy, np.isnan(y)


def Integrate(integrantFunc: callable, f0, t0: float, t1: float, dt: float = None, nSteps: int = 100, 
                     eventFunc: callable = None, eventTol: float = 1e-6, 
                     maxCount: int = 3, initOnly: bool = False, runTillEvent: bool = False,
                     MaxNorm: callable = MaxNormIntegrate, maxNormTol: float = 1e-6):
    if np.isscalar(f0[0]):
        f0 = np.array(f0)
    else:
        f0 = np.array(f0)
        #f0=f0[0]
    finishedByEvent = False
    maxNorm = 2
    stateList = [f0]
    yprime = integrantFunc(t0, f0, [0], [f0])
    primeList = [yprime]
    argList=[t0]

    if dt is None:
        dt = (t1-t0)/nSteps

    dir = np.sign(dt)
    ycur = f0
    curt = t0
    curdt = dt
    err0=1
    if eventFunc is not None: 
        err0 = eventFunc(curt, ycur, [dt], [ycur]) 
        if np.isscalar(err0) and err0 < eventTol:
            return stateList, argList, [primeList, maxNorm, finishedByEvent]
        elif not np.isscalar(err0) and np.all(err0 < eventTol):
            return stateList, argList, [primeList, maxNorm, finishedByEvent]

    count = 0
    factorExp = 0
    while np.any((dir*(t1 - curt)) > 0) or runTillEvent:
        newdt = curdt
        maxNorm = 29
        nsteps = 1
        f0_prev = stateList[-1]
        dt_prev = curdt
        if len(argList) > 1:
            dt_prev = argList[-1] - argList[-2]
            
        while np.max(maxNorm) > 1 and nsteps < 512:
            curdt = newdt
            ytplusFulldt, deltaFull = advanceRK4(tspan=[curt, curt+curdt], f0=deepcopy(ycur), f0_prev=f0_prev, dt_prev=dt_prev, func = integrantFunc)[1:3]
            ytplusHalfdt, deltaHalf1 = advanceRK4(tspan=[curt, curt+curdt/2], f0=deepcopy(ycur), f0_prev=f0_prev, dt_prev=dt_prev, func = integrantFunc)[1:3]
            ytplus2Halfsdt, deltaHalf2 = advanceRK4(tspan=[curt+curdt/2, curt+curdt], f0=deepcopy(ytplusHalfdt), f0_prev=ycur, dt_prev=curdt, func = integrantFunc)[1:3]
            delta2Halfs = deltaHalf1+deltaHalf2  
            maxNorm = MaxNorm(ytplusFulldt, ytplus2Halfsdt, maxNormTol)     #       maxNorm = MaxNorm(deltaFull, delta2Halfs, maxNormTol)
            newdt = curdt/2
            nsteps = np.max((t1 - curt)/newdt)
            if count > maxCount:
                break
            count += 1
            if np.max(maxNorm) > 1:
                factorExp = 0
            f0_prev = deepcopy(ycur)
            dt_prev = curdt
        
        # Richardson Interpolation : dy = Dy_2half + 1/15 (Dy_2half - Dy_full) + O(h^6)
        delta = delta2Halfs + 1./15. * (delta2Halfs - deltaFull)
        nexty = ycur + delta
        nextt = curt + curdt
        yprime = integrantFunc(nextt, nexty, curdt, ycur*0)[0]
        if eventFunc is not None:
            err = eventFunc(nextt, nexty, yprime)
            if np.all(err) < 0:
                if abs(err) < (eventTol*err0) or abs(err) < (eventTol):
                    finishedByEvent = True
                    curt = nextt
                    ycur = nexty
                    break
                else:
                    # curdt /= 2
                    yprimecur = integrantFunc(curt, ycur, ycur*0)[0]
                    errcur = eventFunc(curt, ycur, yprimecur)                   
                    dt = -curdt*errcur/(err-errcur)
                    curdt=dt
                    # errcur+(x-0)*(err-errcur)/(curdt-0)=0
                    # curdt*errcur/(errcur-err)=x
                    continue
        curt = nextt
        ycur = nexty
        curdt = curdt*(sqrt(2)**factorExp)
        factorExp = 1
        
        argList.append(curt)
        stateList.append(deepcopy(ycur))
        
        if initOnly:
            break
        primeList.append(yprime)
        # if eventFunc is not None:
        #     if eventFunc(ycur):
        #         break
    argList.append(curt)
    stateList.append(deepcopy(ycur))
    yprime = integrantFunc(curt, ycur, ycur*0)[0]
    primeList.append(yprime)
    

    return stateList, argList, [primeList, maxNorm, finishedByEvent]
