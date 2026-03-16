japluto@japluto-Legion-Y9000P-IAX10H:~$ docker info | grep "Docker Root Dir"
 Docker Root Dir: /var/lib/docker
japluto@japluto-Legion-Y9000P-IAX10H:~$ watch -n 2 'sudo du -sh /var/lib/docker | tail -n 1'