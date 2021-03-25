import os
import math
import sys
import heapq
import pdb
import numpy as np
import scipy.linalg as sp

import Obj

# class VertexPair:
#     def __init__(self, v1: np.array, v2: np.array):
#         self.v1 = v1
#         self.v2 = v2
#         self.cv = []
#         self.cost = 0.0

class MeshSimplifier:
    # Contains list of all vertices.
    v = []

    # Contains list of all vertices belonging to a face.
    f = []
    faces_valid = True

    # Contains list of valid edges.
    # The key represents the first vertex and the value contains a set
    # of which all vertices in the set are connected by an edge to the key vertex.
    e = None

    def __init__(self, v: list, f: list):
        self.v = v
        self.f = f
        self.build_edges_list()
        return

    # INFO: Each face only contains three edges so build an ADT representing
    # all possible edges.
    def build_edges_list(self):
        assert self.faces_valid == True

        if self.e is None:
            self.e = dict()

        for face in self.f:
            # INFO: Add edge 0 to 1
            if face[0] in self.e:
                self.e[face[0]].add(face[1])
            else:
                self.e[face[0]] = {face[1]}

            # INFO: Add edge 1 to 0. Symmetrical to above.
            if face[1] in self.e:
                self.e[face[1]].add(face[0])
            else:
                self.e[face[1]] = {face[0]}

            # INFO: Add edge 0 to 2
            if face[0] in self.e:
                self.e[face[0]].add(face[2])
            else:
                self.e[face[0]] = {face[2]}

            # INFO: Add edge 2 to 0. Symmetrical to above.
            if face[2] in self.e:
                self.e[face[2]].add(face[0])
            else:
                self.e[face[2]] = {face[0]}

            # INFO: Add edge 1 to 2
            if face[1] in self.e:
                self.e[face[1]].add(face[2])
            else:
                self.e[face[1]] = {face[2]}

            # INFO: Add edge 2 to 1.  Symmetrical to above.
            if face[2] in self.e:
                self.e[face[2]].add(face[1])
            else:
                self.e[face[2]] = {face[1]}

        return

    # INFO: Each face only contains three edges so build an ADT representing
    # all possible valid edges. Valid meaning the vertices share anb edge and that
    # the norm of difference of the vertices is less then the threshold request.
    def build_valid_edges_list(self, threshold) -> list:
        valid_e = dict()

        for v1_idx in self.e:
            for v2_idx in self.e[v1_idx]:
                assert v1_idx < len(self.v), "Index out of bounds! {} >= {}".format(v1_idx, len(self.v))
                assert v2_idx < len(self.v), "Index out of bounds! {} >= {}".format(v2_idx, len(self.v))

                v1 = (v1_idx, np.array(self.v[v1_idx]))
                v2 = (v2_idx, np.array(self.v[v2_idx]))

                if self.is_valid_pair(v1, v2, threshold):
                    if v1_idx in valid_e:
                        valid_e[v1_idx].add(v2_idx)
                    else:
                        valid_e[v1_idx] = {v2_idx}

        return valid_e

    def is_valid_edge(self, vertex_a: (int, np.array), vertex_b: (int, np.array)) -> bool:
        # INFO: Check if vertex a has an edge to vertex b and vice versa.
        is_b_connected_to_a = vertex_b[0] in self.e[vertex_a[0]]
        is_a_connected_to_b = vertex_a[0] in self.e[vertex_b[0]]
        return (is_b_connected_to_a) and (is_a_connected_to_b)

    def is_valid_pair(self, vertex_a: (int, np.array), vertex_b: (int, np.array), threshold) -> bool:
        v_norm = np.linalg.norm(vertex_a[1] - vertex_b[1])
        return self.is_valid_edge(vertex_a, vertex_b) and (v_norm < threshold)

    def triangle_to_plane(self, tri_v: list) -> np.array:
        # INFO: Create PQ and PR which spans plane. Their cross product solves plane normal.
        P, Q, R = tri_v[0], tri_v[1], tri_v[2]
        PQ = np.subtract(P, Q)
        PR = np.subtract(P, R)
        N = np.cross(PQ, PR)

        # INFO: Solve for plane equation constants.
        a = (Q[1] - P[1])*(R[2] - P[2]) - (R[1] - P[1])*(Q[2] - P[2])
        b = (Q[2] - P[2])*(R[0] - P[0]) - (R[2] - P[2])*(Q[0] - P[0])
        c = (Q[0] - P[0])*(R[1] - P[1]) - (R[0] - P[0])*(Q[1] - P[1])
        d = (-1.0) * (a*P[0] + b*P[1] + c*P[2])

        # INFO: Scale plane equations constants such that a^2 + b^2 + c^2 = 1.
        k = 1.0 / math.sqrt(a*a + b*b + c*c)
        ka = k*a
        kb = k*b
        kc = k*c
        kd = k*d
        
        return np.array([ka, kb, kc, kd])

    def calculate_quadric_Kp(self, plane_eqn: np.array) -> np.array:
        Kp = np.dot(plane_eqn, np.transpose(plane_eqn))
        return Kp

    def calculate_vertex_contraction(self, Q: np.array, v1: np.array, v2: np.array) -> np.array:
        # INFO: Build matrices to use in finding new optimal vertex coords.
        R1 = [Q[0][0].item(), Q[0][1].item(), Q[0][2].item(), Q[0][3].item()]
        R2 = [Q[0][1].item(), Q[1][1].item(), Q[1][2].item(), Q[1][3].item()]
        R3 = [Q[0][2].item(), Q[1][2].item(), Q[2][2].item(), Q[2][3].item()]
        R4 = [0, 0, 0, 1]
        M = np.array([R1, R2, R3, R4])

        if np.linalg.cond(M) < (1 / sys.float_info.epsilon):
            # INFO: If matrix is invertible then solve the system to find the optimal
            # coordinates for the new vertex that represent the contraction.
            inv_M = np.linalg.inv(M)
            V = np.transpose(np.array([0, 0, 0, 1]))
            return np.dot(inv_M, V)
        else:
            # INFO: If matrix not invertible return median of vertices.
            return np.divide(np.add(v1 + v2), 2.0)

    def calculate_all_plane_eqns(self) -> dict:
        # INFO: face_plane_eqns: Key = Vertex, Value = Set of all plane eqns for that vertex.
        face_plane_eqns = dict()

        for face in self.f:
            vP, vQ, vR = self.v[face[0]], self.v[face[1]], self.v[face[2]]
            plane_eqn = self.triangle_to_plane([vP, vQ, vR])

            for i in range(0, 3):
                if face[i] in face_plane_eqns:
                    face_plane_eqns[face[i]].append(plane_eqn)
                else:
                    face_plane_eqns[face[i]] = [plane_eqn]

        return face_plane_eqns

    # TODO: Check for correctness.
    def simplify(self, threshold):
        self.faces_valid = False
        min_cost_heap = []

        # INFO: Compute all plane equations.
        face_plane_eqns = self.calculate_all_plane_eqns()

        # INFO: Compute Q for all vertices.
        # q_matrices: Key = Vertex, Value = Sum of all q matrices for that vertex.
        q_matrices = dict()
        for vertex in self.e:
            Q = np.identity(4)
            for plane_eqn in face_plane_eqns[vertex]:
                Kp = self.calculate_quadric_Kp(plane_eqn)
                Q = np.add(Q, Kp)
            q_matrices[vertex] = Q

        # INFO: Debug check.
        assert len(q_matrices) == len(self.v), "The amount of Q matrices should be equal to vertex count! (Q: {}, V: {})".format(len(q_matrices), len(self.v))

        # INFO: 2. Select all valid edges.
        valid_e = self.build_valid_edges_list(threshold)

        # INFO: 3. Compute optimal contractions for all valid edges.
        # INFO: 4. Store those contractions in a heap sorted by cost for each optimal contraction.
        # TODO: Fuked
        for ve1, vlist in valid_e.items():
            for ve2 in vlist:
                v1, v2 = np.array(self.v[ve1]), np.array(self.v[ve2])
                Q = np.add(q_matrices[ve1], q_matrices[ve2])
                V = self.calculate_vertex_contraction(Q, v1, v2)
                short_V = np.array([V[0].item(), V[1].item(), V[2].item()])
                cost = np.dot(np.dot(np.transpose(V), Q), V)
                heapq.heappush(min_cost_heap, (cost, [ve1, ve2, short_V]))

        # TODO: 5. Remove least cost contraction and perform it. Iterate till no valid edges are left
        # to potentially contract.
        valid_e = None
        debug = 0
        deleted_idx = []
        while len(min_cost_heap) > 0:
            print("iteration: {}".format(debug))

            cost_pair = heapq.heappop(min_cost_heap)
            old_V1 = cost_pair[1][0]
            old_V2 = cost_pair[1][1]
            new_V = cost_pair[1][2]
            cost = cost_pair[0]

            print("self.v: {}, min_cost_heap: {}, OV1: {}, OV2: {}".format(len(self.v), len(min_cost_heap), old_V1, old_V2))

            # INFO: If a vertex was deleted skip it.
            if old_V1 in deleted_idx or old_V2 in deleted_idx:
                continue
            if old_V1 >= len(self.v) or old_V2 >= len(self.v):
                continue

            # INFO: Update self.v
            max_idx = max(old_V1, old_V2)
            min_idx = min(old_V1, old_V2)
            self.v[min_idx] = new_V
            del self.v[max_idx]
            new_V_idx = min_idx
            deleted_idx.append(max_idx)

            # INFO: Update self.f
            for i in range(0, len(self.f)):
                f_v1, f_v2, f_v3 = self.f[i][0], self.f[i][1], self.f[i][2]

                # INFO: Check for vertex index that was contracted.
                cond = [f_v1 == max_idx, f_v2 == max_idx, f_v3 == max_idx]
                if any(cond):
                    for j in range(0, 3):
                        self.f[i][j] = len(self.f) if cond[j] else self.f[i][j]
                cond = [f_v1 > max_idx, f_v2 > max_idx, f_v3 > max_idx]
                if any(cond):
                    self.f[i] = [x - 1 for x in self.f[i]]

            # INFO: Update self.e, all edges connected to v2 are now assigned to v1 which was converted 
            # to the contracted vertex.
            self.e[new_V_idx] = self.e[new_V_idx].union(self.e[max_idx])
            self.e[new_V_idx].discard(new_V_idx)
            self.e[new_V_idx].discard(max_idx)
            del self.e[max_idx]

            for key, values in self.e.items():
                values_copy = values.copy()
                for val in values:
                    if val == max_idx:
                        values_copy.discard(val)
                        values_copy.add(new_V_idx)
                    elif val > max_idx:
                        values_copy.discard(val)
                        values_copy.add(val - 1)

                self.e[key] = values_copy

            new_edges = dict()
            for key, values in self.e.items():
                if key > max_idx:
                    new_edges[key - 1] = values
                else:
                    new_edges[key] = values
            self.e = new_edges

            # INFO: Select all valid edges
            valid_e = self.build_valid_edges_list(threshold)
            debug += 1

        return (self.v, self.f)
