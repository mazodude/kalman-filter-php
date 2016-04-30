<?php
/**
* KalmanFilter
* @class
* @author Wouter Bulten
* @see {@link http://github.com/wouterbulten/kalmanjs}
* @version Version: 1.0.0-beta
* @copyright Copyright 2015 Wouter Bulten
* @license GNU LESSER GENERAL PUBLIC LICENSE v3
* @preserve
* @note Ported to PHP by mazodude
*/
class KalmanFilter
{
    /**
    * Create 1-dimensional kalman filter
    * @param  {Number} options.R Process noise
    * @param  {Number} options.Q Measurement noise
    * @param  {Number} options.A State vector
    * @param  {Number} options.B Control vector
    * @param  {Number} options.C Measurement vector
    * @return {KalmanFilter}
    */
    function __construct($R = 1, $Q = 1, $A = 1, $B = 0, $C = 1, $cov = NAN, $x = NAN){
        $this->R = $R; // noise power desirable
        $this->Q = $Q; // noise power estimated

        $this->A = $A;
        $this->B = $B;
        $this->C = $C;
        $this->cov = $cov;
        $this->x = $x; // estimated signal without noise
    }

    /**
    * Filter a new value
    * @param  {Number} z Measurement
    * @param  {Number} u Control
    * @return {Number}
    */
    function filter($z, $u = 0){
        if (is_nan($this->x)) {
            $this->x = (1 / $this->C) * $z;
            $this->cov = (1 / $this->C) * $this->Q * (1 / $this->C);
        }
        else {
            // Compute prediction
            $predX = ($this->A * $this->x) + ($this->B * $u);
            $predCov = (($this->A * $this->cov) * $this->A) + $this->R;

            // Kalman gain
            $K = $predCov * $this->C * (1 / (($this->C * $predCov * $this->C) + $this->Q));

            // Correction
            $this->x = $predX + $K * ($z - ($this->C * $predX));
            $this->cov = $predCov - ($K * $this->C * $predCov);
        }

        return $this->x;
    }

    /**
    * Return the last filtered measurement
    * @return {Number}
    */
    function lastMeasurement() {
        return $this->x;
    }

    /**
    * Set measurement noise Q
    * @param {Number} noise
    */
    function setMeasurementNoise($noise) {
        $this->Q = $noise;
    }

    /**
    * Set the process noise R
    * @param {Number} noise
    */
    function setProcessNoise($noise) {
        $this->R = $noise;
    }

    function saveState() {
        $state = array(
            'R'=>$this->R,
            'Q'=>$this->Q,
            'A'=>$this->A,
            'B'=>$this->B,
            'C'=>$this->C,
            'cov'=>$this->cov,
            'x'=>$this->x
            );
        return json_encode($state);
    }
}
