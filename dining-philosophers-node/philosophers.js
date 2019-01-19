// MATEUSZ ZEMBOL NR INDEKSU 291 586
// ROZWIAZANIA PROBLEMU 5 FILOZOFOW NA 4 ROZNE SPOSOBY

// ----- UTILS -----
const async = require('async');
const now = require('performance-now');

// ----- FORK -----
const Fork = function() {
  this.state = 0;
  this.maxAttempts = 5;
  this.maxAwaiting = 4096;
  return this;
};


// ----- NAIVE VERSION -----
Fork.prototype.acquire = function(callbackSuccess, callbackFailure) {
  let attempts = 0;
  let time = 1;
  const captureFork = function(waitingTime, fork) {
    setTimeout(function() {
      if (fork.state == 1) {
        if (time < fork.maxAwaiting) time *= 2;
        if (attempts > fork.maxAttempts) {
          if (callbackFailure) callbackFailure();
        } else {
          attempts++;
          captureFork(time + Math.random() * 10, fork);
        }
      } else {
        fork.state = 1;
        if (callbackSuccess) callbackSuccess();
      }
    }, waitingTime);
  };
  captureFork(1, this);
};

Fork.prototype.release = function(callback) {
  this.state = 0;
  if (callback) callback();
};

const Philosopher = function(id, forks) {
  this.id = id;
  this.forks = forks;
  this.f1 = id % forks.length;
  this.f2 = (id + 1) % forks.length;
  this.eatingTime = 100 + Math.random() * 100; // miliseconds
  return this;
};

Philosopher.prototype.eat = function(count) {
  const forks = this.forks,
    f1 = this.f1,
    f2 = this.f2,
    id = this.id;

  setTimeout(function() {
    async.waterfall([
      function(callback) {
        forks[f1].release(callback);
      },
      function(callback) {
        forks[f2].release(callback);
      },
      function(callback) {
        philosophers[id].startNaive(count - 1);
      }
    ]);
  }, this.eatTime);
};

Philosopher.prototype.startNaive = function(count) {
  var forks = this.forks,
    f1 = this.f1,
    f2 = this.f2,
    id = this.id;

  if (count != 0) {
    let start = now();
    forks[f1].acquire(
      function() {
        times[id][0] += now() - start;
        start = now();
        forks[f2].acquire(
          function() {
            times[id][1] += now() - start;
            philosophers[id].eat(count);
          },
          function() {
            async.waterfall([
              function(cb) {
                forks[f1].release(cb);
              },
              function(cb) {
                philosophers[id].startNaive(count);
              }
            ]);
          }
        );
      },
      function() {
        philosophers[id].startNaive(count);
      }
    );
  } else {
    console.log(id + 'done!');
  }
};

// ----- A LITTLE BIT SMARTER VERSION -----
Philosopher.prototype.startSmart = function(count) {
  if (this.id % 2 == 0) {
    let tmp = this.f1;
    this.f1 = this.f2;
    this.f2 = tmp;
  }
  this.startNaive(count);
};

// ----- VERSION WITH CONDUCTOR ------

Philosopher.prototype.giveForks = function(count) {
  let philosopher = this;
  let delay = Math.random() * 100;
  setTimeout(function() {
    conductor.release(philosopher, count);
  }, delay);
};

Philosopher.prototype.startConductor = function(count) {
  let id = this.id;
  if (count != 0) conductor.ask(this, count);
  else console.log(id + 'done!');
};

var Conductor = function() {
  this.queue = [];
  return this;
};

Conductor.prototype.ask = function(philosopher, count) {
  let id = philosopher.id,
    f1 = philosopher.f1,
    f2 = philosopher.f2,
    forks = philosopher.forks;
  const start = now();
  if (forks[f1].state === 0 && forks[f2].state === 0) {
    forks[f1].state = 1;
    forks[f2].state = 1;
    times[id][0] += (now() - start) / 10;
    times[id][1] += (now() - start) / 10;
    philosopher.giveForks(count);
  } else {
    this.queue.push([id, count, start]);
  }
};

Conductor.prototype.release = function(philosopher, count) {
  let f1 = philosopher.f1,
    f2 = philosopher.f2,
    forks = philosopher.forks,
    conductor = this;

  forks[f1].state = 0;
  forks[f2].state = 0;

  philosopher.startConductor(count - 1);

  let checkQueue = function() {
    if (conductor.queue.length !== 0) {
      let id = conductor.queue[0][0],
        count = conductor.queue[0][1],
        timestamp = conductor.queue[0][2],
        f1 = philosophers[id].f1,
        f2 = philosophers[id].f2,
        forks = philosophers[id].forks;

      if (forks[f1].state === 0 && forks[f2].state === 0) {
        times[id][0] += (now() - timestamp) / 10;
        times[id][1] += (now() - timestamp) / 10;
        conductor.queue.shift();
        forks[f1].state = 1;
        forks[f2].state = 1;
        philosophers[id].giveForks(count);
        checkQueue();
      }
    }
  };
  checkQueue();
};

// ----- BONUS - ISTANTLY PICK UP BOTH FORKS

Fork.prototype.acquireBoth = function(
  philosopher,
  callbackSuccess,
  callbackFailure
) {
  let attempts = 0;
  let time = 1;
  let fork1 = philosopher.f1;
  let fork2 = philosopher.f2;
  const captureFork = function(waitingTime, fork1, fork2) {
    setTimeout(function() {
      if (fork1.state === 1 || fork2.state === 1) {
        if (time < fork.maxAwaiting) time *= 2;
        if (attempts > fork.maxAttempts) {
          if (callbackFailure) callbackFailure();
        } else {
          attempts++;
          captureFork(Math.floor(time + Math.random() * 100), fork1, fork2);
        }
      } else {
        fork1.state = 1;
        fork2.state = 1;
        if (callbackSuccess) callbackSuccess();
      }
    }, waitingTime);
  };
  captureFork(1, fork1, fork2);
};

Philosopher.prototype.eatBoth = function(count) {
  const forks = this.forks,
    f1 = this.f1,
    f2 = this.f2,
    id = this.id;

  setTimeout(function() {
    async.waterfall([
      function(callback) {
        forks[f1].release(callback);
      },
      function(callback) {
        forks[f2].release(callback);
      },
      function(callback) {
        philosophers[id].startBoth(count - 1);
      }
    ]);
  }, this.eatTime);
};

Philosopher.prototype.startBoth = function(count) {
  let forks = this.forks,
    f1 = this.f1,
    f2 = this.f2,
    id = this.id;

  let philosopher = this;
  if (count != 0) {
    let start = now();
    forks[f1].acquireBoth(
      philosopher,
      function() {
        times[id][0] += now() - start;
        philosopher.eatBoth(count);
      },
      function(cb) {
        philosophers[id].startBoth(count);
      }
    );
  } else {
    console.log(id + 'done!');
  }
};

// ----- STARTING SETTINGS -----
var N = 15;
var forks = [];
var philosophers = [];
const conductor = new Conductor();

// ----- ARRAY WITH TIME -----
const times = Array.apply(null, Array(N)).map(Number.prototype.valueOf, 0);
for (let i = 0; i < N; i++) {
  times[i] = Array.apply(null, Array(2)).map(Number.prototype.valueOf, 0);
}

for (let i = 0; i < N; i++) {
  for (let j = 0; j < 2; j++) {
    times[i][j] = 0;
  }
}

for (var i = 0; i < N; i++) forks.push(new Fork());
for (var i = 0; i < N; i++) philosophers.push(new Philosopher(i, forks));
console.log('starting');
for (var i = 0; i < N; i++) philosophers[i].startBonus(10);

setTimeout(() => {
  for (let i = 0; i < N; i++) {
    for (let j = 0; j < 2; j++) {
      console.log(times[i][j] + ' ');
    }
  }
}, 2000);
