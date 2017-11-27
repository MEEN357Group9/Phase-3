function [T, X, V, A] = Newmark(X0, V0, A0, M, C, K, FN, D)
%
% NEWMARK
%
% Implements Newmark's algorithm for integrating the second-order ODE
%
%       d^2{x}       d{x}
%   [M] ------ + [C] ---- + [K]{x} = {FF}(t,D)
%        dt^2         dt
%
% whose passed parameters are:
%   X0  is a vector containing the initial condition for displacement
%   V0  is a vector containing the initial condition for velocity
%       needed if more than 1 trajectory is to be imposed, size DOFx1
%   A0  is a vector containing the initial condition for acceleration 
%       needed if more than 1 trajectory is to be imposed, size DOFx1
%   M   is a mass matrix
%   C   is a damping matrix
%   K   is a stiffness matrix 
%   FN  is a handle to an arbitrary forcing function with an interface of
%          function [FF,D] = someForcingFunction(t,D)
%       that can be called by
%          [FF,D] = FN(t,D)
%       whose returned vector FF has the same dimension as X and
%       whose data structure D contains data needed to solve FN for FF
%   D   is a data structure needed to solve FN(t,D)
%
% The lower limit of integration comes from D.t_in.
% The upper limit of integration comes from D.t_out.
% The number of integration steps come from D.N.
%
% Vector X0 is a column vector of size 'DOF'.
% Matrices M, C and K are square and symmetric of size 'DOFxDOF'. 
% 'DOF' is the dimension of the problem: the degrees of freedom.
%
% The returned fields contain a vector and three matrices
%   T   is a vector of times at solution    of dimension  D.N+1
%   X   is a vector of position vectors     of dimension (D.N+1)xDOF
%   V   is a vector of velocity vectors     of dimension (D.N+1)xDOF
%   A   is a vector of acceleration vectors of dimension (D.N+1)xDOF
%
% The Newmark algorithm has two parameters: gamma and beta.
% Some commonly used versions of this method include:
%
% version               gamma   beta                 error   stability
% average acceleration  1/2     1/4                  O(h^2)  unconditional 
% linear acceleration   1/2     1/6                  O(h^2)  conditional
% Fox-Goodwin           1/2     1/12                 O(h^4)  conditional
% algorithmic damped    >=1/2   >=((gamma+1/2)^2)/4  O(h)    unconditional
%
% By default, function NEWMARK implements the average acceleration version.
%

% Check the inputs for admissibility.

if ~isstruct(D)
   Error('D, the forcing function data structure, must be a struct.');
end
if D.t_out <= D.t_in
   Error('The final time TN must be greater than the initial time T0.');
end
if D.N < 1
   Error('The number of integration steps must exceed zero.');
end
dof = size(X0,1);
if size(V0,1) ~= dof
   Error('The length of vectors X0 and V0 must be the same.');
end
if size(A0,1) ~= dof
   Error('The length of vectors X0, V0 and A0 must be the same.');
end
[rows, cols] = size(M);
if (rows ~= dof) || (cols ~= dof)
   Error('The mass matrix must have dimension DOFxDOF.');
end
[rows, cols] = size(C);
if (rows ~= dof) || (cols ~= dof)
   Error('The damping matrix must have dimension DOFxDOF.');
end
[rows, cols] = size(K);
if (rows ~= dof) || (cols ~= dof)
   Error('The stiffness matrix must have dimension DOFxDOF.');
end

% Assign the integrator variables for averaged acceleration version.
bet = 1/4;  % beta  is a reserved word
gam = 1/2;  % gamma is a reserved word

% Set the step size.
h = (D.t_out - D.t_in)/D.N; 

% Create the vectors and matrices used by the algorithm.
rhs  = zeros(dof,1);
rhsC = zeros(dof,1);
rhsM = zeros(dof,1);

T = zeros(D.N+1,1);
X = zeros(D.N+1,dof);
V = zeros(D.N+1,dof);
A = zeros(D.N+1,dof);

% Assign initial conditions to the output fields.
T(1) = D.t_in;
for ii=1:dof
   X(1,ii) = X0(ii);
   V(1,ii) = V0(ii);
   A(1,ii) = A0(ii);
end

% Create the matrix for the LU solver and factorize it.
Keff = zeros(dof, dof);
for ii=1:dof
   for jj=1:dof
      Keff(ii,jj) = M(ii,jj)/(bet*h*h) + gam*C(ii,jj)/(bet*h) + K(ii,jj);
   end
end
[L,U] = lu(Keff);

% Integrate over the interval [T0,TN].
for nn=1:D.N
   T(nn+1) = T(nn) + h;
   
   % create the right-hand side mass contribution
   for ii=1:dof
      rhsM(ii) = 0;
      for jj=1:dof
         rhsM(ii) = rhsM(ii) + M(ii,jj)*(X(nn,jj)/(bet*h*h) ...
                  + V(nn,jj)/(bet*h) + (1/(2*bet)-1)*A(nn,jj));
      end
   end
   
   % create the right-hand side damping contribution
   for ii=1:dof
      rhsC(ii) = 0;
      for jj=1:dof
         rhsC(ii) = rhsC(ii) + C(ii,jj)*(gam*X(nn,jj)/(bet*h) ...
                  + (gam/bet-1)*V(nn,jj) + h*(gam/(2*bet)-1)*A(nn,jj));
      end
   end
   
   % assemble the right-hand side vectors
   [rhsF,D] = FN(T(nn+1),D);
   for ii=1:dof
      rhs(ii) = rhsF(ii) + rhsM(ii) + rhsC(ii);
   end
   
   % solve for the displacements  Keff.pos = rhs  where  Keff = L.U
   tmp = L\rhs;  %  solves  L.tmp = rhs  for  tmp
   pos = U\tmp;  %  solves  U.pos = tmp  for  pos
   for ii=1:dof
      X(nn+1,ii) = pos(ii);
   end
   
   % solve for the velocities
   for ii=1:dof
      V(nn+1,ii) = (gam/(bet*h))*(X(nn+1,ii) - X(nn,ii)) ...
                 - (gam/bet-1)*V(nn,ii) - h*(gam/(2*bet)-1)*A(nn,ii);
   end
   
   % solve for the accelerations
   for ii=1:dof
      A(nn+1,ii) = (1/(bet*h*h))*(X(nn+1,ii) - X(nn,ii) - h*V(nn,ii)) ...
                 - (1/(2*bet)-1)*A(nn,ii);
   end
end

end
