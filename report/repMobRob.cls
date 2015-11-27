%==========================================
%
% class for report of mobile robot's practical works 
%
%==========================================

% Declare Class

\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{repAML}


%Load mother's class

%---- Recto seulement ----%
%\LoadClass[11pt,onecolumn,a4paper,oneside,final]{article}
%---- Recto verso ----%

\LoadClass[11pt,onecolumn,a4paper,oneside,openright,final]{article}

%fundamental packages

\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage[british,UKenglish,USenglish,english,american]{babel}

%mathematics packages
\usepackage{array}
\usepackage{amsmath}
\usepackage{amssymb}
\usepackage[version=3]{mhchem}
\usepackage[retainorgcmds]{IEEEtrantools}
\usepackage{siunitx}

%Figures packages
\usepackage{graphicx}
\usepackage{float}
\usepackage{wrapfig}
%\usepackage[center]{subfig}
\usepackage{subcaption}
\usepackage[font={small,it}]{caption}

%names and informations
\newcommand\subtitle[1]{\def\@subtitle{#1}}
\newcommand\teacher[1]{\def\@teacher{#1}}
\newcommand\footauthor[1]{\def\@footauthor{#1}}
\newcommand\semester[1]{\def\@semester{#1}}
\newcommand\cours[1]{\def\@cours{#1}}

%---- Tables ----%
\usepackage{tabularx}
\usepackage{multirow}
\usepackage{multicol}

%--Paper size and layout--%
\usepackage[dvips, 
	    a4paper, 
	    top				=1cm, 
	    bottom			=2cm, 
	    left			=2cm, 
	    headheight		=6mm, 
	    headsep			=10mm, 
	    marginparwidth	=2cm, 
	    marginparsep	=4mm, 
	    heightrounded, 
	    includehead]{geometry}

\renewcommand\@maketitle{%
	\null
	\begin{center}%
	\includegraphics[width=80mm]{fig/logo_EPFL}\par%
	\vspace{2cm}
	\let \footnote \thanks
	\reset@font
	{\usefont{T1}{cmr}{bx}{n}
	\large\sc{\@semester~ - \@cours}\\\vspace{1cm}
	\fontsize{29.86pt}{29.86pt}\bfseries\selectfont\@title\\\vspace{0.5em}
	\LARGE{\@subtitle}\par}%
	%{\LARGE \@title \par}%
	\vskip 6cm% 
	{\large
	\lineskip .5em%
	\begin{tabular}[t]{c}%
	\@author
	\end{tabular}\par}%
	\vskip 1em%
	
	{\large \@date}%
	\vskip 1cm%
	Teacher:\\
	\@teacher\\
	\end{center}%
	\par
	\clearpage}

%---Newcommands---%
\newcommand{\figi}[4]{
   \begin{figure}[H]
   \begin{center}
   \includegraphics[width=#4]{fig/#1}
   \end{center}
   \caption{\label{#2}#3}
   \end{figure}   
} 
