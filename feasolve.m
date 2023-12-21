function [] = feasolve(name)

    %% Model Construction
    close all; clc;
    
    % Filename
    %name = 'truss2';

    % Full name of the inputfile
    inputfile = fullfile(pwd,'Input',sprintf('%s.inp',name));

    %% Read Abaqus Input File
    [Model,Material,BoundaryConditions] = ReadInputAbaqus(inputfile);

    %% Assemble the Stiffness Matrix
    [Model] = Assembly(Model,Material);
    
    %% Partition the Stiffness Matrix
    [Model] = Partition(Model, BoundaryConditions);

    %% Solve the System
    [Model] = SolveSystem(Model);

    %% Plotting
    % Plot the undeformed shape
    %Visualize(name,Model,Material,'Undeformed',1);
    
    % Plot the deformed shape
    Visualize(name,Model,Material,'Deformed',1);
    
    % Plot the deformed shape with stress components
    Visualize(name,Model,Material,'Stress',1);
    
    % Plot the deformed shape with strain components
    Visualize(name,Model,Material,'Strain',1);

    % Make a sparsity plot for the Stiffness Matrix
    %Visualize(name,Model,Material,'Stiffness');
    
    %Play an animation showing movement between deformed and undefomred
    %shapes
    Movie(Model, 15);
    
    %% Post Processing
    Post(name, Model, Material);
end

%% Auxiliary Functions
function [Model,Material,BC] = ReadInputAbaqus(filename)
% Reads Abaqus Input files (.inp) and stores model, material and BC info
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%     filename : name of the input file to be read | string
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%     Model    : Matlab structure that contains all information regarding
%                the model's geometry (see description below)
%     Material : A structure that contains all information regarding the
%                material model used in the finite element analysis (see
%                description below)
%     BC       : A structure that contains all information regarding the
%                displacement or force boundary conditions imposed in 
%                subregions of the model (see description below)
%
%     ---------------------------------------------------------------------
%
%     Model.coordinates  -> Nodal coordinates of each node in the model
%     Model.connectivity -> Element connectivity table
%     Model.area         -> Cross Sectional Area of the elements
%     Model.elType       -> Element type (T2D2, CPS3, CPE3, etc)
%
%     ---------------------------------------------------------------------
%
%     Material.young     -> The Young's modulus of the linear elastic model
%
%     ---------------------------------------------------------------------
%
%     BC.displacement    -> A table with 3 columns: (See following example)
%
%               | Node Number | Degree of Freedom | Value of Displacement |
%               |      23     |          1        |           0.0         |
%               |      23     |          2        |           0.1         |
%               |     112     |          1        |          -2.0         |
%
%     BC.force           -> A table with 3 columns: (See following example)
%
%               | Node Number | Degree of Freedom | Value of Applied Force|
%               |      32     |          1        |           1.5         |
%               |      40     |          2        |           5.0         |
%               |     180     |          1        |          -9.0         |
%
%     BC.dforce          -> A table with 3 columns: (See following example)
%
%               | Element #   |        Type       | Value of Applied Load |
%               |      32     |        'P1'       |           1.5         |
%               |      40     |        'P1'       |           5.0         |
%               |     180     |        'P1'       |          -9.0         |
%
%     ---------------------------------------------------------------------


%   Your coding to define: coordinates, connectivity, area, displacement
%                          young and force
    
    %Open Abaqus File, save file ID
    fileID = fopen(filename);
    
    %FORM COORDINATE MATRIX
    %Parse up to *NODE command
    Parse(fileID, "*node");
    
    %Save the coordinates of each node in coordinates matrix
    %Get first coordinate line
    tline = fgetl(fileID);
    %Initialize coordinate array
    coordinates = [];
    %While the current line is not a command or comment, continue parsing
    while(~startsWith(tline,"*"))
        %Split current line into array of strings
        splits = split(tline,",");
        %Convert the vertical array of strings into a horizontal array of
        %doubles, and append this array to the coordinates array
        coordinates = [coordinates; str2double(splits')];
        %Move to next line
        tline = fgetl(fileID);
    end
    
    %FORM ELEMENT CONNECTIVITY MATRIX
    %Parse up to *ELEMENT command
    boundary = Parse(fileID, "*element");
    %determine element type
    element = split(boundary, {', ','='});
    elType = [element(3) element(5)];
    
    %Save the connectivity of each node in connectivity matrix
    %Get first connectivity line
    tline = fgetl(fileID);
    %Initialize connectivity array
    connectivity = [];
    %While the current line is not a command or comment, continue parsing
    while(~startsWith(tline,"*"))
        %Split current line into array of strings
        splits = split(tline,",");
        %Convert the vertical array of strings into a horizontal array of
        % ints, and append this array to the coordinates array
        connectivity = [connectivity; str2double(splits')];
        %Move to next line
        tline = fgetl(fileID);
    end
    
    %DETERMINE CROSS SECTIONAL AREA
    %Parse up to *SOLID SECTION command
    Parse(fileID, "*solid section");
    %Save next line as a double in the area variable 
    area = str2double(fgetl(fileID));
    
    %DETERMINE YOUNGS MODULUS
    %Parse up to *ELASTIC command
    Parse(fileID, "*elastic");
    %Save next line as a double in the E variable
    elastics = split(fgetl(fileID), {','});
    E = str2double(elastics{1});
    if length(elastics)==2
        V = str2double(elastics{2});
    else
        V = 0;
    end
    
    %DETERMINE BOUNDARY CONDITIONS
    %Parse up to *BOUNDARY command
    Parse(fileID, "*boundary");
    
    %Save the prescribed dispalcements in the displacement table
    %Count number of displacements
    %Initialize counter
    n = 0;
    %Get first line of displacements
    tline = fgetl(fileID);
    %While current line is not a command or comment, continue parsing
    while(~startsWith(tline,"*"))
        %Increment count
        n = n + 1;
        %Split line along commas (Node, 1st DOF, last DOF, displacement)
        splits = split(tline,",");
        %If first and last DOF are different, increase number of
        %displacement lines (x- and y-displacements are counted separately)
        if str2double(splits(2)) ~= str2double(splits(3))
            n = n+1;
        end
        %Move to next line of file
        tline = fgetl(fileID);
    end
    %Initialize displacement table variables
    Node = zeros(n,1);
    DoF = zeros(n,1);
    Displacement = zeros(n,1);
    
    %Return parser to *BOUNDARY condition
    Parse(fileID, "*boundary");
    %Initialize counter
    i = 1;
    %Get first line of displacements
    tline = fgetl(fileID);
    %While current line is not a command or comment, continue parsing
    while(~startsWith(tline,"*"))
        %Split line along commas (Node, 1st DOF, last DOF, displacement)
        splits = split(tline,",");
        %Convert split strings to doubles, save in respective variables
        Node(i) = str2double(splits(1));
        Displacement(i) = str2double(splits(4));
        DoF(i) = str2double(splits(2));
        %If first and last DoF are not the same, save the second DoF as a
        %separate displacement entry
        if str2double(splits(2)) ~= str2double(splits(3))
            %Increment counter to next displacement entry
            i = i+1;
            %Save second DoF in respective displacement entry
            Node(i) = str2double(splits(1));
            Displacement(i) = str2double(splits(4));
            DoF(i) = str2double(splits(3));
        end
        %Increment counter to next displacement entry
        i = i+1;
        %Move to next line of file
        tline = fgetl(fileID);
    end
    %Form displacement table from node, DoF, displacement tables
    displacement = table(Node, DoF, Displacement);
    
    %DETERMINE APPLIED LOADS
    %Parse up to *CLOAD command
    cload = Parse(fileID, "*cload");
    %Initialise force table
    force = table('Size',[0 3], 'VariableTypes', ["double", "double", "double"],'VariableNames',{'Node' 'DoF' 'Load'});
    % determine if *CLOAD command is present
    if ~isnumeric(cload)
        %Count number of loads
        %Initialize counter
        n = 0;
        %Get first line of loads
        tline = fgetl(fileID);
        %While current line is not a command or comment, continue parsing
        while(~startsWith(tline,"*"))
            %Increment count
            n = n + 1;
            %Move to next line of file
            tline = fgetl(fileID);
        end
        %Initialize force table variables
        Node = zeros(n,1);
        DoF = zeros(n,1);
        Load = zeros(n,1);
        
        %Return parser to *CLOAD condition
        Parse(fileID, "*cload");
        %Initialize counter
        i = 1;
        %Get first line of displacements
        tline = fgetl(fileID);
        while(~startsWith(tline,"*"))
            %Split line along commas (Node, 1st DOF, last DOF, displacement)
            splits = split(tline,",");
            %Convert split strings to doubles, save in respective variables
            Node(i) = str2double(splits(1));
            DoF(i) = str2double(splits(2));
            Load(i) = str2double(splits(3));
            %Move to next line of file
            tline = fgetl(fileID);
            %Increment counter
            i = i+1;
        end
        
        %Form force table from node, DoF, loads
        force = table(Node, DoF, Load);
    end 
    
    %Parse to distributed load command, *DLOAD
    dload = Parse(fileID, "*dload");
    %Initialise dforce table
    dforce = table('Size',[0 3], 'VariableTypes', ["double", "string", "double"],'VariableNames',{'Node' 'Type' 'Load'});
    % determine if *DLOAD command is present
    if ~isnumeric(dload)
        %Count number of loads
        %Initialize counter
        n = 0;
        %Get first line of loads
        tline = fgetl(fileID);
        %While current line is not a command or comment, continue parsing
        while(~startsWith(tline,"*"))
            %Increment count
            n = n + 1;
            %Move to next line of file
            tline = fgetl(fileID);
        end
        %Initialize force table variables
        Element = zeros(n,1);
        Type = cell(n,1);
        Load = zeros(n,1);
        
        %Return parser to *dLOAD condition
        Parse(fileID, "*dload");
        %Initialize counter
        i = 1;
        %Get first line of displacements
        tline = fgetl(fileID);
        while(~startsWith(tline,"*"))
            %Split line along commas (Node, type, load)
            splits = split(tline,", ");
            %Convert split strings to doubles, save in respective variables
            Element(i) = str2double(splits(1));
            Type(i) = (splits(2));
            Load(i) = str2double(splits(3));
            %Move to next line of file
            tline = fgetl(fileID);
            %Increment counter
            i = i+1;
        end
        
        %Form force table from node, DoF, loads
        dforce = table(Element, Type, Load);
    end 

    Model.coordinates = coordinates;
    Model.elType = elType;
    Model.connectivity = connectivity;
    Model.area = area;
    
    Material.young = E;
    Material.poisson = V;
    
    BC.displacement = displacement;
    BC.force = force;
    BC.dforce = dforce;
    
    %Close Abaqus file
    fclose(fileID);
end

function [Ke] = Truss2D(Coor1,Coor2,Ae,Ee)
% Creates the stiffness matrix for a 2D truss element
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%     Coor1   : Row vector of the x,y coordinates of the bar's left point
%     Coor2   : Row vector of the x,y coordinates of the bar's right point
%     Ae      : The cross sectional area of the element
%     Ee      : The Young's modulus of the element
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%     Ke      :  The stiffness matrix for the 2D truss element
%
%     ---------------------------------------------------------------------
%
%     Your coding to define Ke (4x4) matrix for the stiffness of the bar
%
%        | K11 K12 K13 K14 | (1x)
%        | K21 K22 K23 K24 | (1y)
%  K =   | K31 K32 K33 K34 | (2x)
%        | K41 K42 K43 K44 | (2y)
%         (1x)(1y)(2x)(2y)
%
    %Calculate length of 2D truss element
    Le = sqrt((Coor1(1)-Coor2(1)).^2 + (Coor1(2)-Coor2(2)).^2);
    %Calculate sine, cosine of 2D truss element
    c = (Coor2(1)-Coor1(1)) / Le;
    s = (Coor2(2)-Coor1(2)) / Le;
    %Construct rotation matrix from angle of bar trig identities
    Te = [c s 0 0; 0 0 c s];
    
    %Calcualte stiffness of member based on cross sectional area, Young's
    %modulus, and element length
    k = Ae * Ee / Le;
    
    %Construct local stiffness matrix of member (1D stiffness along member's
    %local angle)
    Kel = k * [1 -1; -1 1];
    
    %Construct Ke, the 2D element stiffness matrix by rotating the 1D
    %stiffness matrix using the rotation matrix, based on the below formula
    % Te Fe = Kel Te de
    % Fe = (Te' Kel Te) de
    % Fe = Ke de
    Ke = Te'*Kel*Te;
    
end

function [Ke] = Tri2D(Coor1, Coor2, Coor3, Ee, Ve, Th, ElType)
% Creates the stiffness matrix for a 2D linear triangular element
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%     Coor1   : Row vector of the x,y coordinates of the element's 1st point
%     Coor2   : Row vector of the x,y coordinates of the element's 2nd point
%     Coor3   : Row vector of the x,y coordinates of the element's 3rd point
%      Note- nodes are numbered counterclockwise
%     Ee      : The Young's modulus of the element
%     Ve      : The Poisson's ratio of the element
%     Th      : The thickness of the element
%     ElType  : String containing the type of element: "CPS3" or "CPE3"
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%     Ke      :  The stiffness matrix for the 2D truss element
%
%     ---------------------------------------------------------------------   
    %Determine element area using cross products
    vec1 = [Coor2(1)-Coor1(1) Coor2(2)-Coor1(2) 0];
    vec2 = [Coor3(1)-Coor1(1) Coor3(2)-Coor1(2) 0];
    A = 1/2 * norm(cross(vec1, vec2));
    
    %Calculate B, gradient of N
    N1dx = Coor2(2)-Coor3(2);
    N1dy = Coor3(1)-Coor2(1);
    N2dx = Coor3(2)-Coor1(2);
    N2dy = Coor1(1)-Coor3(1);
    N3dx = Coor1(2)-Coor2(2);
    N3dy = Coor2(1)-Coor1(1);
    
    B = 1/(2*A)* [N1dx 0    N2dx 0    N3dx 0;
                  0    N1dy 0    N2dy 0    N3dy;
                  N1dy N1dx N2dy N2dx N3dy N3dx];
              
    %Determine D matrix type: plane stress or plane strain
    switch ElType
        case "CPS3"
        %plane stress
        D = Ee / (1-Ve^2)* [1 Ve 0; Ve 1 0; 0 0 (1-Ve)/2];
        case "CPE3"
        %plane strain
        D = Ee /((1+Ve)*(1-2*Ve)) * [1-Ve Ve 0; Ve 1-Ve 0; 0 0 (1-2*Ve)/2];
    end
    %Calculate element stiffness matrix
    Ke = Th*A*B'*D*B;
end

function [Ke] = Quad2D(Coor1, Coor2, Coor3, Coor4, Ee, Ve, Th, ElType)
% Creates the stiffness matrix for a 2D linear quad element
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%     Coor1   : Row vector of the x,y coordinates of the element's 1st point
%     Coor2   : Row vector of the x,y coordinates of the element's 2nd point
%     Coor3   : Row vector of the x,y coordinates of the element's 3rd point
%     Coor4   : Row vector of the x,y coordinates of the element's 4th point
%      Note- nodes are numbered counterclockwise
%     Ee      : The Young's modulus of the element
%     Ve      : The Poisson's ratio of the element
%     Th      : The thickness of the element
%     ElType  : String containing the type of element: "CPS4" or "CPE4"
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%     Ke      :  The stiffness matrix for the 2D truss element
%
%     ---------------------------------------------------------------------

    switch ElType
        case "CPS4"
            %plane stress
            D = Ee / (1-Ve^2)* [1 Ve 0; Ve 1 0; 0 0 (1-Ve)/2];
        case "CPE4"
            %plane strain
            D = Ee /((1+Ve)*(1-2*Ve)) * [1-Ve Ve 0; Ve 1-Ve 0; 0 0 (1-2*Ve)/2];
    end

    %Gaussian Points
    xiG = 1/sqrt(3)*[-1 1];
    etaG = 1/sqrt(3)*[-1 1];
        
    %Initialize empty Ke matrix
    Ke = zeros(8,8);
    %Form coordinate matrix
    coor = [Coor1; Coor2; Coor3; Coor4];

    %Loop through each of the four guassian points
    for i = 1:2
        for j = 1:2
            %Wi*Wj*det(Je(xii,etaj))*BeT(xii,etaj)*De*Be(xii,etaj)
            Je = jacobian(xiG(i),etaG(j),coor);
            Be = bMatrix(xiG(i),etaG(j),Je);
            
            %Add gaussian point contribution to Ke matrix
            Ke = Ke+ Be'*D*Be*det(Je);

        end
    end
    
    Ke = Th*Ke;
    
    %Function to calculate jacobian given xi and eta values
    function [Je] = jacobian(xi,eta,coor)
        %A is [dN1/dxi  dN2/dxi  dN3/dxi  dN4/dxi;
        %      dN1/deta dN2/deta dN3/deta dN4/deta]
        A = 1/4* [eta-1 1-eta 1+eta -eta-1;
                  xi-1  -xi-1 1+xi  1-xi];
        %Jacobian Matrix
        Je = A*coor;
    end
    %Function to calculate b matrix given xi and eta values
    function [Be] = bMatrix(xi,eta,Je)
        %dNi = [dN1/dx dN2/dx dN3/dx dN4/dx;
        %       dN1/dy dN2/dy dN3/dy dN4/dy]
        %dNi = inv(Je) * A, where A is same as in jacobian function
        dNi = (Je) \ (0.25*[eta-1 1-eta 1+eta -eta-1;
                            xi-1  -xi-1 1+xi  1-xi]);
        Be = [dNi(1,1) 0        dNi(1,2) 0        dNi(1,3) 0        dNi(1,4) 0;
              0        dNi(2,1) 0        dNi(2,2) 0        dNi(2,3) 0        dNi(2,4);
              dNi(2,1) dNi(1,1) dNi(2,2) dNi(1,2) dNi(2,3) dNi(1,3) dNi(2,4) dNi(1,4)];
    end
end

function [Model] = Assembly(Model,Material)
% Assembles the global stiffness matrix
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%     Model    : Matlab structure that contains all information regarding
%     Material : A structure that contains all information regarding the
%                material model used in the finite element analysis
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%     Model    : Matlab structure that contains all information regarding
%                the model's geometry as well as the global stiffness mtrx


    % Include code to define
    %       ndof : The number of degrees of freedom of the model
    %       noel : The number of elements in the model
    %       K    : The global stiffness matrix of the model
        
    %Determine number of DoF in the model, based on the number of nodes.
    %Each node has two degrees of freedom (2D space)
    ndof = length(Model.coordinates(:,1))*2;
    
    %Determine number of elements in the model, based on the length of the
    %Model.connectivity array
    noel = length(Model.connectivity(:,1));
    
    %Create array of element stiffness matricies, based on type of element:
    %triangular or truss, for now.
    switch Model.elType{1}
        case {'CPE4','CPS4'}
            Ksize = 8;
            Kelements = zeros(Ksize,Ksize,noel);
            
            for i = 1:noel
                %get ith element
                element = Model.connectivity(i,:);
                %Get x,y coordinates nodes of ith element
                Coor1 = Model.coordinates(element(2),2:3);
                Coor2 = Model.coordinates(element(3),2:3);
                Coor3 = Model.coordinates(element(4),2:3);
                Coor4 = Model.coordinates(element(5),2:3);
                
                Ee = Material.young;
                Ve = Material.poisson;
                Th = Model.area;
                ElType = Model.elType{1};
                
                Kelements(:,:,i) = Quad2D(Coor1, Coor2, Coor3, Coor4, Ee, Ve, Th, ElType);
            end
            
        case {'CPE3','CPS3'}
            %Triangular element type, either plane stress or strain
            Ksize = 6;
            Kelements = zeros(Ksize,Ksize,noel);
            
            for i = 1:noel
                %get ith element
                element = Model.connectivity(i,:);
                %Get x,y coordinates nodes of ith element
                Coor1 = Model.coordinates(element(2),2:3);
                Coor2 = Model.coordinates(element(3),2:3);
                Coor3 = Model.coordinates(element(4),2:3);

                Ee = Material.young;
                Ve = Material.poisson;
                Th = Model.area;
                ElType = Model.elType{1};
                
                Kelements(:,:,i) = Tri2D(Coor1, Coor2, Coor3, Ee, Ve, Th, ElType);
            end
           
        case 'T2D2'
            %Truss element type
            Ksize = 4;
            %Create empty matrix to contain all element stiffness matricies
            Kelements = zeros(Ksize,Ksize,noel);
            
            %Fill Kelements matrix with element stiffness matricies for each
            %element listed in Model.connectivity
            for i = 1:noel
                %Get ith element
                element = Model.connectivity(i,:);
                %Get x,y coordinates for first node of ith element, from the
                %Model.coordinates matrix
                Coor1 = Model.coordinates(element(2),2:3);
                %Get x,y coordinates for second node of ith element, from the
                %Model.coordinates matrix
                Coor2 = Model.coordinates(element(3),2:3);
                %Get the cross-sectional area of the element
                Ae = Model.area;
                %Get the Young's modulus of the element
                Ee = Material.young;
                %Save the element connectivity matrix in the ith page of Kelements
                Kelements(:,:,i) = Truss2D(Coor1,Coor2,Ae,Ee);
            end
    end
    %Create empty global stiffness matrix, with two rows and columns for
    %each node (or one row and column for each degree of freedom)
    K = zeros(ndof, ndof);
    
    %For each element, insert the components of the element stiffness
    %matrix into the corresponding place in the global stiffness matrix
    %First determine number of nodes in each element
    numNodes = length(Model.connectivity(1,:))-1;
    for i = 1:noel
        %Create empty array containing global stiffness matrix ranges
        %corresponding to each node in the element
        ranges = zeros([numNodes 2]);
        %Populate the ranges array using nodes in element
        for j = 1:numNodes
            %Determine jth node of element
            node = Model.connectivity(i,j+1);
            %Calculate the range corresponding to this node in the global
            %stiffness matrix (Node 1 is 1:2, Node 2 is 3:4, etc...)
            ranges(j,:) = [node*2-1 node*2];
        end
        
        %Iterate through each node-node pair in element
        for j = 1:numNodes
            for k = 1:numNodes
                %Extract connectivity elements for each 2x2 part of the element
                %connectivity matrix
                KeSplit = Kelements((j*2-1):(j*2), (k*2-1):(k*2), i);
                
                %Add the connectivity element 2x2 array to the correct range of the
                %global stiffness matrix
                K(ranges(j,:),ranges(k,:)) = K(ranges(j,:),ranges(k,:))+KeSplit;
            end
        end
 
    end
    
	% Packing variables back to Model
    Model.ndof = ndof;
    Model.noel = noel;
    Model.K = K;
end

function [Model] = Partition(Model,BC)
% Uses the partition method to enforce the boundary conditions
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%   Model   : Matlab structure that contains all information regarding
%             the model's geometry as well as the stiffness matrix
%   BC      : A sturcture that contains all information regarding the 
%             displacement or force boundary conditions imposed in
%             subregions of the model (see description below)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%   Model   : Matlab structure that contains all information regarding
%             the model's geometry, the stiffness matrix as well as the
%             partitioned matricies (KEE, KFE, .... FE, dE....)
% 
    
    K = Model.K;
    % Your coding to define KFF, KEE, KFE, KEF, FF, dE
    
    %Convert the essential node list to a sorted list, by element number.
    %This assists in ordering the arrays correctly.
    essentialNodes = table2array(sortrows(BC.displacement,1));

    %Determine the number of total, free, and essential node/dof pairs
    numNodes = 2*length(Model.coordinates(:,1));
    numEssential = length(essentialNodes(:,1));
    numFree = numNodes-numEssential;
    
    %Create the list of free node/dof pairs
    %initialise the free node/dof list
    freeNodes = zeros(numFree,3);
    %Initialise the counter for positioning within freeNodes
    j = 1;
    %Iterate through all the nodes
    for i=1:numNodes
        %Determine the node and dof corresponding to the index
        node = round(i/2);
        dof = 2-mod(i,2);
        %If the node/dof pair is not an essential node, add the pair to the
        %list of free nodes
        if ~ismember([node dof],essentialNodes(:,1:2),'rows')
            freeNodes(j,1:2)=[node dof];
            j = j+1;
        end
    end

    %define sizes of each matrix.
    %initialise each stiffness matrix with the correct size 
    KEE = zeros(numEssential, numEssential);
    KFF = zeros(numFree, numFree);
    KFE = zeros(numFree, numEssential);
    KEF = zeros(numEssential, numFree);
    
    %Fill in the values of each partitioned stiffness matrix using the
    %values in K. Each loop iterates through the corresponding lists of
    %essential or free nodes and inserts the correct value from K
    
    %Populate KEE, KEF
    for i = 1:numEssential
        %Identify node and location of first essential node
        node1 = essentialNodes(i,1:2);
        locK1 = 2*node1(1)+node1(2)-2;
        %KEE
        for j = 1:numEssential
            %Identify node and location of second essential node
            node2 = essentialNodes(j,1:2);
            locK2 = 2*node2(1)+node2(2)-2;
            %Set value in KEE to value in K
            KEE(i,j) = K(locK1, locK2);
        end
        %KEF
        for j = 1:numFree
            %Identify node and location of second free node
            node2 = freeNodes(j,1:2);
            locK2 = 2*node2(1)+node2(2)-2;
            %Set value in KEE to value in K
            KEF(i,j) = K(locK1, locK2);
        end
    end
    %Populate KFF, KFE
    for i = 1:numFree
        %Identify node and location of first free node
        node1 = freeNodes(i,1:2);
        locK1 = 2*node1(1)+node1(2)-2;
        %KFF
        for j = 1:numFree
            %Identify node and location of second free node
            node2 = freeNodes(j,1:2);
            locK2 = 2*node2(1)+node2(2)-2;
            %Set value in KEE to value in K
            KFF(i,j) = K(locK1, locK2);
        end
        %KFE
        for j = 1:numEssential
            %Identify node and location of second essential node
            node2 = essentialNodes(j,1:2);
            locK2 = 2*node2(1)+node2(2)-2;
            %Set value in KEE to value in K
            KFE(i,j) = K(locK1, locK2);
        end
    end 
    
    % Create the dE vector from the essentialNodes list. The last column of
    % essentialNodes lists the displacement for that node/dof pair
    dE = essentialNodes(:,3);
    
    % Extract forces applied to free nodes from boundary conditions
    % Initialise FF vector
    FF = freeNodes(:,3);
    % Apply point forces to FF vector
    if ~isempty(BC.force)
        % Convert the BC.force table to a sorted array
        forceArray = table2array(sortrows(BC.force));
        % Create a vector giving the location of each node/dof pair in
        % freeNodes within the list of applied forces. If a free node does not
        % have an applied force associated with it, the 'location' will be
        % zero.
        [~,loc] = ismember(freeNodes(:,1:2),forceArray(:,1:2),'rows');
        %Iterate through each free node/dof pair, and if it has an associated
        %force, set the force in FF corresponding to that node/dof pair to the
        %force value.
        for i=1:numFree
            if loc(i)~=0
                FF(i) = forceArray(loc(i),3);
            end
        end
    end
    %Apply distributed forces to FF vector (works for triangular for now)
    if ~isempty(BC.dforce)
       for i = 1:height(BC.dforce)
           %Determine charactaristics of distributed force
           element = BC.dforce{i, 'Element'};
           type = BC.dforce{i,'Type'}{1};
           load = BC.dforce{i,'Load'};
           face = str2double(type(2)); %Works for P* Loads
           
           %Determine nodes force is applied to
           elNodes = Model.connectivity(element,2:end);
           nodes = [elNodes(face) elNodes(mod(face,length(elNodes))+1)];
           nodeLocs = [ Model.coordinates(nodes(1),2:end);
                        Model.coordinates(nodes(2),2:end)];
           %Find vector representation of edge
           dx = nodeLocs(2,1) - nodeLocs(1,1);
           dy = nodeLocs(2,2) - nodeLocs(1,2);
           
           %Find inverse normal vector (left hand perpendicular vector)
           n = [0 -1;1 0]*[dx;dy];
           
           %Decompose pressure into x & y coords, scale for edge
           px = 0.5 * n(1) * load * Model.area;
           py = 0.5 * n(2) * load * Model.area;
           
           %Create force array for loaded nodes
           forceArray = [nodes(1) 1 px;
                         nodes(1) 2 py;
                         nodes(2) 1 px;
                         nodes(2) 2 py];
           % Create a vector giving the location of each node/dof pair in
           % freeNodes within the list of applied forces. If a free node does not
           % have an applied force associated with it, the 'location' will be
           % zero.
           [~,loc] = ismember(freeNodes(:,1:2), forceArray(:,1:2),'rows');
           %Iterate through each free node/dof pair, and if it has an associated
           %force, add the force in FF corresponding to that node/dof pair to the
           %force value.
           for i=1:numFree
               if loc(i)~=0
                   FF(i) = FF(i) + forceArray(loc(i),3);
               end
           end
       end
    end
        
    %Save a list of the free and essential nodes, for use in visualization 
    %& postprocessing
    Model.freeNodes = freeNodes;
    Model.essentialNodes = essentialNodes;
    
    Model.KEE = KEE;
    Model.KEF = KEF;
    Model.KFE = KFE;
    Model.KFF = KFF;
    Model.FF  = FF;
    Model.dE  = dE;
end

function [Model] = SolveSystem(Model)
%Uses the partition method to enforce the boundary conditions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%   Model   : Matlab structure that contains all information regarding the
%             model's geometry as well as the stiffness matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%   Model   : Matlab structure that contains all infromation regarding the
%             model's geometry, the stiffness matrix, as well as the 
%             partitioned matricies (KEE, KFE,....FF, dE...)
%             Now it also contains the solution under
%               
%             Model.Solution.dF (unknown displacements)
%             Model.Solution.FE (unknown reaction forces)
%  ----------------------------------------------------------------------
    
    Solution.dF = Model.KFF\(Model.FF-Model.KEF'*Model.dE);
    Solution.FE = Model.KEF*Solution.dF+Model.KEE*Model.dE;
    Model.Solution = Solution;
end

function [] = Visualize(filename,Model,Material,identifier,defscale)
% A function that generates plots to visualize the model
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%     filename  : String, used to name the figures and set plot titles
%     Model     : Matlab structure containing the model's geometry
%     K         : The global stiffness matrix
%     identifier: String; What type of plot to generate
%                 Currently: 'Undeformed' or 'Stiffness'
%                 The first plots the undeformed shape of the structure
%                 whereas the second generates a sparsity plot for the
%                 stiffness matrix
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%
%     ---------------------------------------------------------------------

    %Check if output folder is required, and create it if it doesn't exist
    FolderName =[pwd,'/Results'];
    if exist(FolderName,'dir')==0
        mkdir(FolderName)
    end
    path = [pwd, '/Results/',filename];
    % What type of plot to generate
    switch identifier
        
        % Creating a plot of the undeformed configuration
        case {'Undeformed','UNDEFORMED'}
            %Initialise and clear figure window
            figure(1)
            clf
            hold on
            %Separate x,y coordinates into vectors
            x = Model.coordinates(:,2);
            y = Model.coordinates(:,3);
            %Plot x,y points as individual data points, with formatting
            plot(x,y,'o','MarkerEdgeColor','black','MarkerFaceColor','red')
            %determine element type (truss or continuous)
            switch Model.elType{1}
                case {'T2D2'}
                    %Loop through each line of connectivity matrix
                    for i = 1:length(Model.connectivity(:,1))
                        %Save first and second nodes of element
                        n1 = Model.connectivity(i,2);
                        n2 = Model.connectivity(i,3);
                        %Plot a line from first to second node of element
                        plot([x(n1),x(n2)], [y(n1),y(n2)],'black')
                    end
                case {'CPS3', 'CPE3'}
                    T = Model.connectivity(:,2:4);
                    trisurf(T, x, y, zeros(size(x,1),1), 'LineWidth',2);
                case {'CPS4','CPE4'}
                    %Implement undeformed quad elements
                    patch('Faces',Model.connectivity(:,2:5),'Vertices',Model.coordinates(:,2:3),'FaceColor','green')
                    
            end
            %Include a title
            title(['Undeformed Shape of ' filename]);
            %Set axies equal, to preserve shape
            axis('equal')
            title(['Undeformed Structure (' filename ')']);
            hold off
            saveas(gcf,[path '_undf'],'png')
        
        %Plot deformed and undeformed configurations side-by-side
        case {'Deformed','DEFORMED'}
            %Initialise and clear figure window
            figure(2)
            clf          
                        
            %Separate x,y coordinates into vectors
            x = Model.coordinates(:,2);
            y = Model.coordinates(:,3);
            
            %Determine optimal orientation of plots, horizontal or vertical
            if (max(x)-min(x)) > (max(y)-min(y))
                %horizontal orientation, two rows, one column
                graph_row=2;
                graph_col=1;
            else
                %vertical orientation, one row, two colums
                graph_row=1;
                graph_col=2;
            end
            
            %Plot undeformed, on the left
            subplot(graph_row,graph_col,1)
            hold on
            %Plot x,y points as individual data points, with formatting
            plot(x,y,'o','MarkerEdgeColor','black','MarkerFaceColor','red')
            %determine element type (truss or continuous)
            switch Model.elType{1}
                case {'T2D2'}
                    %Loop through each line of connectivity matrix
                    for i = 1:length(Model.connectivity(:,1))
                        %Save first and second nodes of element
                        n1 = Model.connectivity(i,2);
                        n2 = Model.connectivity(i,3);
                        %Plot a line from first to second node of element
                        plot([x(n1),x(n2)], [y(n1),y(n2)],'black')
                    end
                case {'CPS3', 'CPE3'}
                    T = Model.connectivity(:,2:4);
                    trisurf(T, x, y, zeros(size(x,1),1), 'LineWidth',2);
                case {'CPS4','CPE4'}
                    %Implement undeformed quad elements
                    patch('Faces',Model.connectivity(:,2:5),'Vertices',Model.coordinates(:,2:3),'FaceColor','green')
    
            end
            %set axies equal, to preserve shape
            axis('equal')
            %print a title
            title(['Undeformed Structure (' filename ')']);
            hold off
            
            %Plot deformed plot, on the right
            subplot(graph_row,graph_col,2)
            hold on
            %Offset each of the free nodes by it's displacement from
            %Model.Solution.dF
            for i=1:size(Model.Solution.dF)
                %Find global node and DOF corresponding to index in dF
                node = Model.freeNodes(i,1);
                dof = Model.freeNodes(i,2);
                %Offset x or y location
                if(dof==1)
                    x(node) = x(node) + Model.Solution.dF(i)*defscale;
                else
                    y(node) = y(node) + Model.Solution.dF(i)*defscale;
                end
            end
            %offset each of the essential nodes by it's displacement from
            %Model.dE
            for i=1:size(Model.dE)
                %Find global node and DOF corresponding to index in dF
                node = Model.essentialNodes(i,1);
                dof = Model.essentialNodes(i,2);
                %Offset x or y location
                if(dof==1)
                    x(node) = x(node) + Model.dE(i)*defscale;
                else
                    y(node) = y(node) + Model.dE(i)*defscale;
                end
            end
            %Plot x,y points as individual data points, with formatting
            plot(x,y,'o','MarkerEdgeColor','black','MarkerFaceColor','red')
            %determine element type (truss or continuous)
            switch Model.elType{1}
                case {'T2D2'}
                    %Loop through each line of connectivity matrix
                    for i = 1:length(Model.connectivity(:,1))
                        %Save first and second nodes of element
                        n1 = Model.connectivity(i,2);
                        n2 = Model.connectivity(i,3);
                        %Plot a line from first to second node of element
                        plot([x(n1),x(n2)], [y(n1),y(n2)],'black')
                    end
                case {'CPS3', 'CPE3'}
                    T = Model.connectivity(:,2:4);
                    trisurf(T, x, y, zeros(size(x,1),1), 'LineWidth',2);
                case {'CPS4','CPE4'}
                    %Implement undeformed quad elements
                    patch('Faces',Model.connectivity(:,2:5),'Vertices',[x,y],'FaceColor','green')

            end
            %set axies equal, to preserve shape
            axis('equal')
            %include a title
            title(['Deformed Structure (' filename ')']);
            hold off
            
            %save plot output
            saveas(gcf,[path '_def'],'png')
            
        case {'Stress','STRESS'}
            %Separate x,y coordinates into vectors
            x = Model.coordinates(:,2);
            y = Model.coordinates(:,3);
            
            %Offset each of the free nodes by it's displacement from
            %Model.Solution.dF
            for i=1:size(Model.Solution.dF)
                %Find global node and DOF corresponding to index in dF
                node = Model.freeNodes(i,1);
                dof = Model.freeNodes(i,2);
                %Offset x or y location
                if(dof==1)
                    x(node) = x(node) + Model.Solution.dF(i)*defscale;
                else
                    y(node) = y(node) + Model.Solution.dF(i)*defscale;
                end
            end
            %offset each of the essential nodes by it's displacement from
            %Model.dE
            for i=1:size(Model.dE)
                %Find global node and DOF corresponding to index in dF
                node = Model.essentialNodes(i,1);
                dof = Model.essentialNodes(i,2);
                %Offset x or y location
                if(dof==1)
                    x(node) = x(node) + Model.dE(i)*defscale;
                else
                    y(node) = y(node) + Model.dE(i)*defscale;
                end
            end
            
            ElType = Model.elType{1};
            %Determine optimal orientation of plots, horizontal or vertical
            if (max(x)-min(x)) > (max(y)-min(y))
                %horizontal orientation, two rows, one column
                graph_row=3;
                graph_col=1;
            else
                %vertical orientation, one row, two colums
                graph_row=1;
                graph_col=3;
            end
            switch ElType
                case {"CPS3","CPE3"}
                    %Find stresses
                    [S,~] = SETri2D(Model,Material);
                    
                    T = Model.connectivity(:,2:4);
                    StressLabels = ["S11" "S22" "S12"];
                    
                    figure(3)
                    for i=1:length(StressLabels)
                        subplot(graph_row,graph_col,i)
                        trisurf(T, x, y, zeros(size(x,1),1), S(i,:), 'LineWidth',2);
                        title(StressLabels(i))
                        axis('equal')
                        view(2)
                        colormap jet
                        colorbar
                    end
                case {"CPE4","CPS4"}
                    %Find Stresses
                    [S,~] = SEQuad2D(Model,Material);
                    StressLabels = ["S11" "S22" "S12"];
                    figure(3)
                    for i=1:length(StressLabels)
                        subplot(graph_row,graph_col,i)
                        patch('Faces',Model.connectivity(:,2:5),'Vertices',[x,y],'FaceColor','flat','CData',S(i,:))
                        title(StressLabels(i))
                        axis('equal')
                        view(2)
                        colormap jet
                        colorbar
                    end
            end
        case {'Strain','STRAIN'}
            %Find displaced coordinates
            %Separate x,y coordinates into vectors
            x = Model.coordinates(:,2);
            y = Model.coordinates(:,3);
            %Offset each of the free nodes by it's displacement from
            %Model.Solution.dF
            for i=1:size(Model.Solution.dF)
                %Find global node and DOF corresponding to index in dF
                node = Model.freeNodes(i,1);
                dof = Model.freeNodes(i,2);
                %Offset x or y location
                if(dof==1)
                    x(node) = x(node) + Model.Solution.dF(i)*defscale;
                else
                    y(node) = y(node) + Model.Solution.dF(i)*defscale;
                end
            end
            %offset each of the essential nodes by it's displacement from
            %Model.dE
            for i=1:size(Model.dE)
                %Find global node and DOF corresponding to index in dF
                node = Model.essentialNodes(i,1);
                dof = Model.essentialNodes(i,2);
                %Offset x or y location
                if(dof==1)
                    x(node) = x(node) + Model.dE(i)*defscale;
                else
                    y(node) = y(node) + Model.dE(i)*defscale;
                end
            end
            %Determine optimal orientation of plots, horizontal or vertical
            if (max(x)-min(x)) > (max(y)-min(y))
                %horizontal orientation, two rows, one column
                graph_row=3;
                graph_col=1;
            else
                %vertical orientation, one row, two colums
                graph_row=1;
                graph_col=3;
            end
            ElType = Model.elType{1};
            switch ElType
                case {"CPS3","CPE3"}
                    %Find strains
                    [~,E] = SETri2D(Model,Material);
                    T = Model.connectivity(:,2:4);
                    StrainLabels = ["E11" "E22" "\gamma12"];
                    
                    figure(4)
                    for i=1:length(StrainLabels)
                        subplot(graph_row,graph_col,i)
                        trisurf(T, x, y, zeros(size(x,1),1), E(i,:), 'LineWidth',2);
                        title(StrainLabels(i))
                        axis('equal')
                        view(2)
                        colorbar
                        colormap jet
                    end
                case {"CPS4","CPE4"}
                    %Find Strains
                    [~,E] = SEQuad2D(Model,Material);
                    StrainLabels = ["E11" "E22" "\gamma12"];
                    figure(4)
                    for i=1:length(StrainLabels)
                        subplot(graph_row,graph_col,i)
                        patch('Faces',Model.connectivity(:,2:5),'Vertices',[x,y],'FaceColor','flat','CData',E(i,:))
                        title(StrainLabels(i))
                        axis('equal')
                        view(2)
                        colormap jet
                        colorbar
                    end
            end
            
        % Generating sparsity plot of the global stiffness matrix
        case {'Stiffness','STIFFNESS'}
            % Include code to plot the sparsity of the stiffness matrix
            % here
            figure(5)
            clf
            spy(Model.K);
            saveas(gcf,[path '_spy'],'png')
            
    end

end

function [] = Movie(Model, frames)
% A function that generates a short animation to visualize deformation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%     Model     : Matlab structure containing the model's geometry
%     frames    : The number of frames to animate
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%
%
% Note: issue with animation right now. The plot reshapes the axies when
% iterating through deformation, so stationary points may appear to move
% relative to the animation frame, although relative deformation is
% correct.
%     ---------------------------------------------------------------------

    %Configure figure window and plotting settings
    figure(6)
    clf
    ax=gca;
    ax.NextPlot = 'replaceChildren';
    %Initialise array of frames
    F(frames) = struct('cdata',[],'colormap',[]);
    
    %Save initial x,y values
    x_i = Model.coordinates(:,2);
    y_i = Model.coordinates(:,3);
    
    %Save final x,y values
    x_f = x_i;
    y_f = y_i;

    %Offset each of the free nodes by it's displacement from
    %Model.Solution.dF
    for i=1:size(Model.Solution.dF)
        %Find global node and DOF corresponding to index in dF
        node = Model.freeNodes(i,1);
        dof = Model.freeNodes(i,2);
        %Offset x or y location
        if(dof==1)
            x_f(node) = x_i(node) + Model.Solution.dF(i);
        else
            y_f(node) = y_i(node) + Model.Solution.dF(i);
        end
    end
    %offset each of the essential nodes by it's displacement from
    %Model.dE
    for i=1:size(Model.dE)
        %Find global node and DOF corresponding to index in dF
        node = Model.essentialNodes(i,1);
        dof = Model.essentialNodes(i,2);
        %Offset x or y location
        if(dof==1)
            x_f(node) = x_i(node) + Model.dE(i);
        else
            y_f(node) = y_i(node) + Model.dE(i);
        end
    end
    
    %interpolate between final, initial values
    span = linspace(0,1,frames);
    X = x_i + (x_f - x_i)*span;
    Y = y_i + (y_f - y_i)*span;
    
    %Loop through all frames
    for i=1:frames
        hold on
        %Plot x,y points as individual data points, with formatting
        plot(X(:,i),Y(:,i),'o','MarkerEdgeColor','black','MarkerFaceColor','red')
        
        %determine element type (truss or continuous)
        switch Model.elType{1}
            case {'T2D2'}
                %Loop through each line of connectivity matrix
                for j = 1:length(Model.connectivity(:,1))
                    %Save first and second nodes of element
                    n1 = Model.connectivity(j,2);
                    n2 = Model.connectivity(j,3);
                    %Plot a line from first to second node of element
                    plot([X(n1,i),X(n2,i)], [Y(n1,i),Y(n2,i)],'black')
                end
            case {'CPS3', 'CPE3'}
                T = Model.connectivity(:,2:4);
                trisurf(T, X(:,i), Y(:,i), zeros(size(x,1),1), 'LineWidth',2);
            case {'CPS4','CPE4'}
                %Implement undeformed quad elements
                patch('Faces',Model.connectivity(:,2:5),'Vertices',[X(:,i) Y(:,i)],'FaceColor','green')
                
        end
        
        
        hold off
        axis('equal')
        %Draw, record frame
        drawnow limitrate
        F(i)=getframe;
        clf
    end
    %Play movie twice, looping backwards
    movie(F,-2);
    
end

function [] = Post(filename, Model, Material)
% A function that generates plaintext output of deformation, reaction
% forces, and stresses and strains in the model.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input:
%     filename  : String, used to name the figures and set plot titles
%     Model     : Matlab structure containing the model's geometry
%     Material  : A structure that contains all information regarding the
%                 material model used in the finite element analysis (see
%                 description below)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Output:
%
%     ---------------------------------------------------------------------

    %Check if output folder is required, and create it if it doesn't exist
    FolderName =[pwd,'/Results'];
    if exist(FolderName,'dir')==0
        mkdir(FolderName)
    end
    path = [pwd, '/Results/',filename,'.txt'];
    
    %Initialise Node, displacement, and reaction force matricies
    Nodes = Model.coordinates(:,1);
    U = zeros([size(Nodes,1),2]);
    RF = zeros([size(Nodes,1),2]);
    
    %Extract displacement and reaction forces for each node
    for i = 1:size(Nodes)
        %Loop through x- and y- dof
        for dof = 1:2
            %Create node/dof pair array
            node = [i dof];
            %Determine if node/dof pair is is free or essential
            [~,loc] = ismember(node,Model.essentialNodes(:,1:2),'rows');
            %If location is not zero, the node/dof pair exists in the
            %essentialNodes array. 
            if(loc ~= 0)
                %Record the displacement and reaction forces from the
                %appropriate arrays in Model
                U(i,dof) = Model.essentialNodes(loc,3);
                RF(i,dof) = Model.Solution.FE(loc);
            %If location is zero, the node/dof pair is free.
            else
                %Find the location of node/dof pair in the freeNodes array
                [~,loc] = ismember(node,Model.freeNodes(:,1:2),'rows');
                %Record the displacement and reaction forces from the
                %appropriate arrays in Model                
                U(i,dof) = Model.Solution.dF(loc);
                RF(i,dof) = Model.FF(loc);
            end
        end
    end
    
    %Print displacement data to a formatted text file
    fileID = fopen(path,'w');
    %Print node data, formatted
    title = ["Node","UX","UY","RFX","RFY"];
    output = [Nodes'; U'; RF'];
    fprintf(fileID,'%7s %15s %15s %15s %15s \n',title);
    fprintf(fileID,'%7d %15.4E %15.4E %15.4E %15.4E \n',output);
    
    %Compute stress and strain for each element
    switch Model.elType{1}
        case {'CPE4','CPS4'}
            Elements = Model.connectivity(:,1);
            [S,E] = SEQuad2D(Model,Material);
            %Print Stress Data to Formatted Text File
            %print spacing lines
            fprintf(fileID,'\n \n','');
            %Print element data, formatted
            title = ["Element","E11","E22","E12","S11","S22","S12"];
            output = [Elements'; E; S];
            fprintf(fileID,'%7s %15s %15s %15s %15s %15s %15s \n',title);
            fprintf(fileID,'%7d %15.4E %15.4E %15.4E %15.4E %15.4E %15.4E\n',output);
            
        
        case {'CPE3','CPS3'}
            Elements = Model.connectivity(:,1);
            [S,E] = SETri2D(Model,Material);
            %Print Stress Data to Formatted Text File
            %print spacing lines
            fprintf(fileID,'\n \n','');
            %Print element data, formatted
            title = ["Element","E11","E22","E12","S11","S22","S12"];
            output = [Elements'; E; S];
            fprintf(fileID,'%7s %15s %15s %15s %15s %15s %15s \n',title);
            fprintf(fileID,'%7d %15.4E %15.4E %15.4E %15.4E %15.4E %15.4E\n',output);
            
        case 'T2D2'
            %Initialise element, stress, strain matricies
            Elements = Model.connectivity(:,1);
            E = zeros(size(Elements));
            S = zeros(size(Elements));
            
            %Loop through each element
            for i = 1:size(Elements)
                %Determine first, second nodes of element
                node1 = Model.connectivity(i,2);
                node2 = Model.connectivity(i,3);
                %Determine initial location of element nodes
                pos1_i = Model.coordinates(node1,2:3);
                pos2_i = Model.coordinates(node2,2:3);
                %Calculate initial length of element
                L_i = sqrt( (pos1_i(1)-pos2_i(1)).^2 + (pos1_i(2)-pos2_i(2)).^2);
                
                %Offset element locations using displacement data
                pos1_f = pos1_i + U(node1,:);
                pos2_f = pos2_i + U(node2,:);
                %Calculate final length of element
                L_f = sqrt( (pos1_f(1)-pos2_f(1)).^2 + (pos1_f(2)-pos2_f(2)).^2);
                
                %calculate strain of element
                E(i) = (L_f - L_i)/(L_i);
                %Calculate stress of element using Young's Modulus
                S(i) = E(i) * Material.young;
                
            end
            %Print Stress Data to Formatted Text File
            %print spacing lines
            fprintf(fileID,'\n \n','');
            %Print element data, formatted
            title = ["Element","E","S"];
            output = [Elements'; E'; S'];
            fprintf(fileID,'%7s %15s %15s \n',title);
            fprintf(fileID,'%7d %15.4E %15.4E \n',output);
    end
    fclose(fileID);
    
end

%Function returns stress and strain for each element in a CPE3 or CPS3
%Model
function [S,E] = SETri2D(Model,Material)
    %Initialise Node, displacement matricies
    Nodes = Model.coordinates(:,1);
    U = zeros([size(Nodes,1),2]);
    
    %Extract displacement and reaction forces for each node
    for i = 1:size(Nodes)
        %Loop through x- and y- dof
        for dof = 1:2
            %Create node/dof pair array
            node = [i dof];
            %Determine if node/dof pair is is free or essential
            [~,loc] = ismember(node,Model.essentialNodes(:,1:2),'rows');
            %If location is not zero, the node/dof pair exists in the
            %essentialNodes array. 
            if(loc ~= 0)
                %Record the displacement and reaction forces from the
                %appropriate arrays in Model
                U(i,dof) = Model.essentialNodes(loc,3);
            %If location is zero, the node/dof pair is free.
            else
                %Find the location of node/dof pair in the freeNodes array
                [~,loc] = ismember(node,Model.freeNodes(:,1:2),'rows');
                %Record the displacement and reaction forces from the
                %appropriate arrays in Model                
                U(i,dof) = Model.Solution.dF(loc);
            end
        end
    end

    Elements = Model.connectivity(:,1);
    E = zeros([3, length(Elements)]);
    S = zeros([3, length(Elements)]);

    for i = 1:size(Elements)
        %Calculate Strain, B*displacement
        %Determine nodes, coordinates
        node1 = Model.connectivity(i,2);
        node2 = Model.connectivity(i,3);
        node3 = Model.connectivity(i,4);

        Coor1 = Model.coordinates(node1,2:3);
        Coor2 = Model.coordinates(node2,2:3);
        Coor3 = Model.coordinates(node3,2:3);

        %Determine element area using cross products
        vec1 = [Coor2(1)-Coor1(1) Coor2(2)-Coor1(2) 0];
        vec2 = [Coor3(1)-Coor1(1) Coor3(2)-Coor1(2) 0];
        A = 1/2 * norm(cross(vec1, vec2));
        %Calculate B Matrix
        N1dx = Coor2(2)-Coor3(2);
        N1dy = Coor3(1)-Coor2(1);
        N2dx = Coor3(2)-Coor1(2);
        N2dy = Coor1(1)-Coor3(1);
        N3dx = Coor1(2)-Coor2(2);
        N3dy = Coor2(1)-Coor1(1);
        B = 1/(2*A)* [N1dx 0    N2dx 0    N3dx 0;
            0    N1dy 0    N2dy 0    N3dy;
            N1dy N1dx N2dy N2dx N3dy N3dx];
        %extract element displacement data
        de = [U(node1,1);
            U(node1,2);
            U(node2,1);
            U(node2,2);
            U(node3,1);
            U(node3,2)];
        %strain is B*displacment
        E(:,i)=B*de;

        %Calculate stress, D*strain
        %Construct D matrix
        Ee = Material.young;
        Ve = Material.poisson;
        Th = Model.area;
        ElType = Model.elType{1};
        %Determine D matrix type: plane stress or plane strain
        switch ElType
            case "CPS3"
                %plane stress, CPS3
                D = Ee / (1-Ve^2)* [1 Ve 0; Ve 1 0; 0 0 (1-Ve)/2];
            case "CPE3"
                %plane strain, CPE3
                D = Ee /((1+Ve)*(1-2*Ve)) * [1-Ve Ve 0; Ve 1-Ve 0; 0 0 (1-2*Ve)/2];
        end
        %stress is D*strain
        S(:,i) = D*E(:,i);
    end
end

%function returns stress and strain for each element in a CPE4 or CPS4
function [S,E] = SEQuad2D(Model,Material)
    %Initialise Node, displacement matricies
    Nodes = Model.coordinates(:,1);
    U = zeros([size(Nodes,1),2]);
    
    %Extract displacement and reaction forces for each node
    for i = 1:size(Nodes)
        %Loop through x- and y- dof
        for dof = 1:2
            %Create node/dof pair array
            node = [i dof];
            %Determine if node/dof pair is is free or essential
            [~,loc] = ismember(node,Model.essentialNodes(:,1:2),'rows');
            %If location is not zero, the node/dof pair exists in the
            %essentialNodes array. 
            if(loc ~= 0)
                %Record the displacement and reaction forces from the
                %appropriate arrays in Model
                U(i,dof) = Model.essentialNodes(loc,3);
            %If location is zero, the node/dof pair is free.
            else
                %Find the location of node/dof pair in the freeNodes array
                [~,loc] = ismember(node,Model.freeNodes(:,1:2),'rows');
                %Record the displacement and reaction forces from the
                %appropriate arrays in Model                
                U(i,dof) = Model.Solution.dF(loc);
            end
        end
    end

    Elements = Model.connectivity(:,1);
    E = zeros([3, length(Elements)]);
    S = zeros([3, length(Elements)]);
    
    for el = 1:size(Elements)
        node1 = Model.connectivity(el,2);
        node2 = Model.connectivity(el,3);
        node3 = Model.connectivity(el,4);
        node4 = Model.connectivity(el,5);
        
        Coor1 = Model.coordinates(node1,2:3);
        Coor2 = Model.coordinates(node2,2:3);
        Coor3 = Model.coordinates(node3,2:3);
        Coor4 = Model.coordinates(node4,2:3);
        
        %Construct D matrix
        Ee = Material.young;
        Ve = Material.poisson;
        ElType = Model.elType{1};
        %Determine D matrix type: plane stress or plane strain
        switch ElType
            case "CPS4"
                %plane stress, CPS3
                D = Ee / (1-Ve^2)* [1 Ve 0; Ve 1 0; 0 0 (1-Ve)/2];
            case "CPE4"
                %plane strain, CPE3
                D = Ee /((1+Ve)*(1-2*Ve)) * [1-Ve Ve 0; Ve 1-Ve 0; 0 0 (1-2*Ve)/2];
        end
        
        %Gaussian Points
        xiG = 1/sqrt(3)*[-1 1];
        etaG = 1/sqrt(3)*[-1 1];
        
        %Form coordinate matrix
        coor = [Coor1; Coor2; Coor3; Coor4];
        
        %extract element displacement data
        de = [U(node1,1);
            U(node1,2);
            U(node2,1);
            U(node2,2);
            U(node3,1);
            U(node3,2);
            U(node4,1);
            U(node4,2)];
        
        %Loop through each of the four guassian points
        for i = 1:2
            for j = 1:2
                Je = jacobian(xiG(i),etaG(j),coor);
                Be = bMatrix(xiG(i),etaG(j),Je);
                
                %Add Gaussian point component to strain value
                E(:,el) = E(:,el)+1/4*Be*de;
            end
        end
        %Calculate stress, D*E
        S(:,el) = D*E(:,el);
    end

    %Function to calculate jacobian given xi and eta values
    function [Je] = jacobian(xi,eta,coor)
        %A is [dN1/dxi  dN2/dxi  dN3/dxi  dN4/dxi;
        %      dN1/deta dN2/deta dN3/deta dN4/deta]
        A = 1/4* [eta-1 1-eta 1+eta -eta-1;
                  xi-1  -xi-1 1+xi  1-xi];
        %Jacobian Matrix
        Je = A*coor;
    end
    %Function to calculate b matrix given xi and eta values
    function [Be] = bMatrix(xi,eta,Je)
        %dNi = [dN1/dx dN2/dx dN3/dx dN4/dx;
        %       dN1/dy dN2/dy dN3/dy dN4/dy]
        %dNi = inv(Je) * A, where A is same as in jacobian function
        dNi = (Je) \ (0.25*[eta-1 1-eta 1+eta -eta-1;
                            xi-1  -xi-1 1+xi  1-xi]);
        Be = [dNi(1,1) 0        dNi(1,2) 0        dNi(1,3) 0        dNi(1,4) 0;
              0        dNi(2,1) 0        dNi(2,2) 0        dNi(2,3) 0        dNi(2,4);
              dNi(2,1) dNi(1,1) dNi(2,2) dNi(1,2) dNi(2,3) dNi(1,3) dNi(2,4) dNi(1,4)];
    end
end

%Function parses through fileID file until a line beginning with the target
%string is reached. The parser is left on the next line after target. The
%function matches with target case insensitive.
function [tline] = Parse(fileID, target)
    %Start at beginning of file
    frewind(fileID);
    %Get first line
    tline = fgetl(fileID);
    %While the current line doesn't start with the target string, continue 
    % parsing file
    while(~isnumeric(tline) && ~startsWith(tline,target,'IgnoreCase',true))
        tline=fgetl(fileID);
    end
end