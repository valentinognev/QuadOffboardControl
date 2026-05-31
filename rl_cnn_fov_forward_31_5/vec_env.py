"""Vectorized environment wrapper — runs N workers in parallel subprocesses, each with M envs."""
from __future__ import annotations

import multiprocessing as mp

import numpy as np

from env_swarm_polygon import EnvConfig, SwarmSearchPolygonEnv


def _worker(remote: mp.connection.Connection, cfg: EnvConfig, worker_id: int, num_envs: int):
    envs = [SwarmSearchPolygonEnv(cfg) for _ in range(num_envs)]
    
    while True:
        cmd, data = remote.recv()
        if cmd == 'step':
            # data: list or array of actions for each env in this worker
            next_obs_list, reward_list, done_list, info_list = [], [], [], []
            for i, env in enumerate(envs):
                obs, reward, terminated, truncated, info = env.step(data[i])
                done = terminated or truncated
                if done:
                    # auto-reset
                    obs, info_reset = env.reset(seed=info.get('_next_seed', worker_id * 100000 + i * 10000 + int(info.get('episode', 0))))
                    info['_terminal'] = True
                    info.update({f'terminal_{k}': v for k, v in info_reset.items() if k != '_terminal'})
                else:
                    info['_terminal'] = False
                next_obs_list.append(obs)
                reward_list.append(reward)
                done_list.append(done)
                info_list.append(info)
            remote.send((next_obs_list, reward_list, done_list, info_list))
            
        elif cmd == 'reset':
            # data is a base seed
            obs_list, info_list = [], []
            for i, env in enumerate(envs):
                obs, info = env.reset(seed=data + i)
                obs_list.append(obs)
                info_list.append(info)
            remote.send((obs_list, info_list))
            
        elif cmd == 'get_critic_obs':
            remote.send([env.get_critic_observation() for env in envs])
            
        elif cmd == 'get_frontier':
            remote.send([env.get_frontier_potential() for env in envs])
            
        elif cmd == 'get_frontier_wp':
            remote.send([env._compute_frontier_waypoints() for env in envs])
            
        elif cmd == 'get_guidance':
            remote.send([env.get_guidance_vector() for env in envs])
            
        elif cmd == 'get_marginal':
            remote.send([env.get_marginal_contribution() for env in envs])
            
        elif cmd == 'get_territorial':
            remote.send([env.get_territorial_fraction() for env in envs])
            
        elif cmd == 'get_info':
            remote.send({
                'max_steps': envs[0].max_steps,
                'max_agents': envs[0].max_agents,
                'actor_obs_spec': envs[0].actor_obs_spec,
                'critic_obs_spec': envs[0].critic_obs_spec,
            })
            
        elif cmd == 'close':
            remote.close()
            break


class VecEnv:
    """Run multiple SwarmSearchPolygonEnv in parallel subprocesses."""

    def __init__(self, cfg: EnvConfig, num_workers: int = 4, num_envs_per_worker: int = 1):
        self.num_workers = num_workers
        self.num_envs_per_worker = num_envs_per_worker
        self.total_envs = num_workers * num_envs_per_worker
        self.cfg = cfg
        self.remotes: list[mp.connection.Connection] = []
        self.procs: list[mp.Process] = []

        for i in range(num_workers):
            parent_conn, child_conn = mp.Pipe()
            proc = mp.Process(target=_worker, args=(child_conn, cfg, i, num_envs_per_worker), daemon=True)
            proc.start()
            child_conn.close()
            self.remotes.append(parent_conn)
            self.procs.append(proc)

        # Get env info from first worker
        self.remotes[0].send(('get_info', None))
        info = self.remotes[0].recv()
        self.max_agents = info['max_agents']
        self.max_steps = info['max_steps']
        self.actor_obs_spec = info['actor_obs_spec']
        self.critic_obs_spec = info['critic_obs_spec']

    def reset_all(self, base_seed: int = 0):
        for i, remote in enumerate(self.remotes):
            remote.send(('reset', base_seed + i * self.num_envs_per_worker))
        results = [r.recv() for r in self.remotes]
        
        obs_list = []
        info_list = []
        for res_obs, res_info in results:
            obs_list.extend(res_obs)
            info_list.extend(res_info)
            
        return self._stack_obs(obs_list), info_list

    def step(self, actions: np.ndarray):
        """actions: (total_envs, max_agents, action_dim)"""
        for i, remote in enumerate(self.remotes):
            start = i * self.num_envs_per_worker
            end = start + self.num_envs_per_worker
            remote.send(('step', actions[start:end]))
            
        results = [r.recv() for r in self.remotes]
        obs_list = []
        reward_list = []
        done_list = []
        info_list = []
        for res_obs, res_reward, res_done, res_info in results:
            obs_list.extend(res_obs)
            reward_list.extend(res_reward)
            done_list.extend(res_done)
            info_list.extend(res_info)
            
        rewards = np.stack(reward_list)  # (total_envs, max_agents)
        dones = np.array(done_list)  # (total_envs,)
        return self._stack_obs(obs_list), rewards, dones, info_list

    def get_critic_observations(self):
        for r in self.remotes:
            r.send(('get_critic_obs', None))
        res = []
        for r in self.remotes:
            res.extend(r.recv())
        return res

    def get_frontier_potentials(self):
        for r in self.remotes:
            r.send(('get_frontier', None))
        res = []
        for r in self.remotes:
            res.extend(r.recv())
        return np.stack(res)  # (total_envs, max_agents)

    def get_frontier_waypoints(self):
        for r in self.remotes:
            r.send(('get_frontier_wp', None))
        res = []
        for r in self.remotes:
            res.extend(r.recv())
        return np.stack(res)  # (total_envs, max_agents, 3)

    def get_marginal_contributions(self):
        for r in self.remotes:
            r.send(('get_marginal', None))
        res = []
        for r in self.remotes:
            res.extend(r.recv())
        return np.stack(res)  # (total_envs, max_agents)

    def get_territorial_fractions(self):
        for r in self.remotes:
            r.send(('get_territorial', None))
        res = []
        for r in self.remotes:
            res.extend(r.recv())
        return np.stack(res)  # (total_envs, max_agents)

    def get_guidance_vectors(self):
        for r in self.remotes:
            r.send(('get_guidance', None))
        res = []
        for r in self.remotes:
            res.extend(r.recv())
        return np.stack(res)  # (total_envs, max_agents, 2)

    def _stack_obs(self, obs_list: list[dict]) -> dict[str, np.ndarray]:
        keys = obs_list[0].keys()
        return {k: np.stack([o[k] for o in obs_list]) for k in keys}

    def close(self):
        for r in self.remotes:
            r.send(('close', None))
        for p in self.procs:
            p.join(timeout=5)

