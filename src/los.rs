extern crate nalgebra as na;

use na::{SMatrix, SVector, Unit};

#[allow(non_snake_case)]
pub struct LOS<const D: usize> {
    delta: f64,
}

impl<const D: usize> Default for LOS<D> {
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl<const D: usize> LOS<D> {
    pub fn new(lookahead_distance: f64) -> LOS<D> {
        LOS {
            delta: lookahead_distance,
        }
    }

    pub fn update(
        &mut self,
        pos: &SVector<f64, D>,
        pos_d: &SVector<f64, D>,
        tau: &Unit<SVector<f64, D>>,
    ) -> Unit<SVector<f64, D>> {
        let proj =
            SMatrix::<f64, D, D>::identity() - tau.into_inner() * tau.into_inner().transpose();
        let pos_err = pos - pos_d;
        let mu = Unit::new_normalize(self.delta * tau.into_inner() - proj * pos_err);

        return mu;
    }

    pub fn set_gains(&mut self, lookahead_distance: f64) {
        self.delta = lookahead_distance;
    }

    pub fn get_gains(&self) -> f64 {
        self.delta
    }
}

pub fn ssa(ang: f64) -> f64 {
    let pi = core::f64::consts::PI;
    modulo(ang + pi, 2.0 * pi) - pi
}

fn modulo(m: f64, n: f64) -> f64 {
    (m % n + n) % n
}
