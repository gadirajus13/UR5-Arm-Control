% Finds multiple manipulability measures of a given Jacobian
% INPUTS
% J: a 6 x 6 Manipulability Matrix
% measure: a string argument (see code below for details) 
% OUTPUTS
% mu: corresponding manipulability metric

function mu = manipulability(J,measure)

    lambda = eig((J.')*J);          % finding eigenvalues of "A" matrix in SVD
    sigma = sqrt(lambda);
    
    if (measure == "sigmamin")      % Finds minimum sigma value
        mu = min(sigma);
    elseif (measure == "detjac")    % Finds determinant of Jacobian
        mu = det(J);
    elseif (measure == "invcond")   % Finds the inverse condition number
        mu = min(sigma)/max(sigma);
    
    else
        errordlg("Measure input is Invalid");
    end
    
end