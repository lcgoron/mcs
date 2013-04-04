
" use current indent level to set it for new line
set autoindent
" intelligently indent new line based on previous
set smartindent
" set tab size
set tabstop=2
" set indent size
set shiftwidth=2
" cursor jumps briefly to match a closing or opening statement
set showmatch
" show statusline w/ current cursor position
set ruler
" search for text as you enter it
set incsearch

set number
set noignorecase
set smartcase
set smartindent
set tabstop=2
set shiftwidth=2
set expandtab
set hlsearch
set mouse=a

" insert new line without entering insert mode
map <S-Enter> O<Esc>
map <CR> o<Esc>

" functional keys
nnoremap <F5> <ESC>:w<CR>:make<CR>:cwindow<CR>
nnoremap <F6> <ESC>:cn<CR>
nnoremap <F7> <ESC>:cp<CR>
nnoremap <F8> <ESC>:!./a.out<CR>
nnoremap <F12> :let etws=@/<Bar>:%s/\s\+$//e<Bar>:let @/=etws<Bar>:nohl<CR>

" easier navigation inside braces/parenthesis
map [[ ?[<CR>
map ]] /]<CR>
map {{ ?{<CR>
map }} /}<CR>
map (( ?(<CR>
map )) /)<CR>

" cursor always in middle of screen
:set scrolloff=20

" single-line comments
map ,# :s/^\(.\)/# \1/<CR> " Bash
map ,/ :s/^\(.\)/\/\/ \1/<CR> " C
map ,% :s/^\(.\)/% \1/<CR> " Prolog

" single-line uncomments
map ,u :s/^\([#"%!;]\\|\/\/\\|--\) \(.\)/\2/<CR>:nohlsearch<CR>

" wrapping comments
map ,* :s/^\(.*\)$/\/\* \1 \*\//<CR>:nohlsearch<CR> " C

autocmd BufNewFile,BufRead *.launch set syntax=xml

" enable file type detection
filetype plugin indent on

" ---------------------------------------------------------------------------- "



" Remember last editing position. "
au BufReadPost *
 \  if line("'\"") > 0
 \|   if line("'\"") <= line("$")
 \|     exe("norm '\"")
 \|   else
 \|     exe("norm $")
 \|   endif
 \| endif

" Enable syntax highlighting by default. "
syntax on

" Enable cross-style highlighting of cursor. "
 "set cursorcolumn
 "set cursorline

" Highlighting which moves with the cursor. "
 ":hi CursorLine   cterm=NONE ctermbg=darkcyan ctermfg=white guibg=darkred guifg=white
 ":hi CursorColumn cterm=NONE ctermbg=darkred  ctermfg=white guibg=darkred guifg=white
 ":nnoremap <Leader>c :set cursorline! cursorcolumn!<CR>
 ":nnoremap <Leader>c :set cursorline! <CR>

" Change the highlight of search matches. "
hi Search ctermbg=Green ctermfg=Black

