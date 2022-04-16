# Image to contain quick requirements
# or requirements that should be easily editable
FROM tb3-base

# Config files
COPY .aliases /home/$USER_NAME/.aliases

COPY .tmux.conf /home/$USER_NAME/.tmux.conf

COPY .zshrc /home/$USER_NAME/.zshrc

WORKDIR /home/$USER_NAME/

CMD ["zsh"]
