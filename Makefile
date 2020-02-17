BASE_IMAGE_MD5:=$$(find dockerfiles/dev-env/base -type f | LANG=C sort | xargs -I{} cat {} | openssl sha1 -r | awk '{print $$1}')
WINDOWS_IMAGE_MD5:=$$(find dockerfiles/dev-env/base dockerfiles/dev-env/windows -type f | LANG=C sort | xargs -I{} cat {} | openssl sha1 -r | awk '{print $$1}')
LINUX_IMAGE_MD5:=$$(find dockerfiles/dev-env/base dockerfiles/dev-env/linux -type f | LANG=C sort | xargs -I{} cat {} | openssl sha1 -r | awk '{print $$1}')
BASE_IMAGE_NAME:=nakayamaatsushi/fq1150-vsm-build-base
WINDOWS_IMAGE_NAME:=nakayamaatsushi/fq1150-vsm-build-windows
LINUX_IMAGE_NAME:=nakayamaatsushi/fq1150-vsm-build-linux

ci_image: update_ci
	# build base image
	time (docker images -q ${BASE_IMAGE_NAME}:${BASE_IMAGE_MD5} | wc -l | xargs -I {} [ {} -ne 0 ] || \
	docker pull ${BASE_IMAGE_NAME}:${BASE_IMAGE_MD5} || \
	docker build \
		--file ./dockerfiles/dev-env/base/Dockerfile \
		--force-rm \
		--no-cache \
		--squash \
		--tag ${BASE_IMAGE_NAME}:${BASE_IMAGE_MD5} \
		./ )
	time docker tag ${BASE_IMAGE_NAME}:${BASE_IMAGE_MD5} ${BASE_IMAGE_NAME}:latest
	time (docker pull ${BASE_IMAGE_NAME}:${BASE_IMAGE_MD5} \
	|| (docker push ${BASE_IMAGE_NAME}:${BASE_IMAGE_MD5} \
		&& docker push ${BASE_IMAGE_NAME}:latest))

	# build Linux Development Environment image
	time (docker images -q ${LINUX_IMAGE_NAME}:${LINUX_IMAGE_MD5} | wc -l | xargs -I {} [ {} -ne 0 ] || \
	docker pull ${LINUX_IMAGE_NAME}:${LINUX_IMAGE_MD5} || \
	docker build \
		--file ./dockerfiles/dev-env/linux/Dockerfile \
		--build-arg BASE_IMAGE_MD5=${BASE_IMAGE_MD5} \
		--force-rm \
		--no-cache \
		--squash \
		--tag ${LINUX_IMAGE_NAME}:${LINUX_IMAGE_MD5} \
		./ )
	time docker tag ${LINUX_IMAGE_NAME}:${LINUX_IMAGE_MD5} ${LINUX_IMAGE_NAME}:latest
	time (docker pull ${LINUX_IMAGE_NAME}:${LINUX_IMAGE_MD5} \
	|| (docker push ${LINUX_IMAGE_NAME}:${LINUX_IMAGE_MD5} \
		&& docker push ${LINUX_IMAGE_NAME}:latest))

	# build Windows Development Environment image
	time (docker images -q ${WINDOWS_IMAGE_NAME}:${WINDOWS_IMAGE_MD5} | wc -l | xargs -I {} [ {} -ne 0 ] || \
	docker pull ${WINDOWS_IMAGE_NAME}:${WINDOWS_IMAGE_MD5} || \
	docker build \
		--file ./dockerfiles/dev-env/windows/Dockerfile \
		--build-arg BASE_IMAGE_MD5=${BASE_IMAGE_MD5} \
		--build-arg LINUX_IMAGE_MD5=${LINUX_IMAGE_MD5} \
		--force-rm \
		--no-cache \
		--squash \
		--tag ${WINDOWS_IMAGE_NAME}:${WINDOWS_IMAGE_MD5} \
		./ )
	time docker tag ${WINDOWS_IMAGE_NAME}:${WINDOWS_IMAGE_MD5} ${WINDOWS_IMAGE_NAME}:latest
	time (docker pull ${WINDOWS_IMAGE_NAME}:${WINDOWS_IMAGE_MD5} \
	|| (docker push ${WINDOWS_IMAGE_NAME}:${WINDOWS_IMAGE_MD5} \
		&& docker push ${WINDOWS_IMAGE_NAME}:latest))

	time docker image prune -f || true

update_ci:
	rm -f .env
	touch .env
	echo "WINDOWS_TAG=${WINDOWS_IMAGE_MD5}" >> .env
	echo "LINUX_TAG=${LINUX_IMAGE_MD5}" >> .env
